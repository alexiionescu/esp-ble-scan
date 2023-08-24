use embassy_time::Instant;
use embedded_io::blocking::Write;
use esp_wifi::{ble::controller::BleConnector, EspWifiInitialization};
use hal::radio::Bluetooth;
use log::{info, warn};

const PACKET_HEADER_LENGTH: usize = 4;
const PACKET_TYPE_COMMAND: u8 = 0x01;
const PACKET_TYPE_EVENT: u8 = 0x04;

const OPCODE_RESET: u16 = 0x0C03;
const OPCODE_SET_EVENT_MASK: u16 = 0x0C01;
const OPCODE_LE_SET_EVENT_MASK: u16 = 0x2001;
const OPCODE_SET_SCAN_PARAMS: u16 = 0x200B;
const OPCODE_SET_SCAN_ENABLE: u16 = 0x200C;
// const OPCODE_ADD_TO_WHITELIST: u16 = 0x2011;

const EVENT_COMMAND_COMPLETE: u8 = 0x0E;

/// Interval and window are in units of the BLE timing unit of 0.625
/// milliseconds. 30 milliseconds / .625 happens to be 0x30 in hexidecimal.
const SCAN_PARAM_SCAN_INTERVAL: u16 = 0x0030;
const SCAN_PARAM_SCAN_WINDOW: u16 = SCAN_PARAM_SCAN_INTERVAL;
/// No filtering
const SCAN_PARAM_FILTER_ALLOW_ALL: u8 = 0x00;
/// Only report events for addresses that have been added to the list
const SCAN_PARAM_FILTER_ALLOW_LISTED: u8 = 0x01;

/// Handles Bluetooth LE scanning for the Ble. It supports a single Ble.
pub struct Scanner {
    ble: BleConnector<'static>,
    pub packets: u32,
    pub packets_size: usize,
}

impl Scanner {
    pub fn new(init: &EspWifiInitialization, bluetooth: Bluetooth) -> Self {
        Self {
            ble: BleConnector::new(init, bluetooth),
            packets: 0,
            packets_size: 0,
        }
    }

    /// Initializes the scanner
    pub fn init(&mut self) {
        info!("Start Ble Scanner...");
        self.write_cmd(&hci_reset());

        // info!("Filtering unwanted events");
        self.write_cmd(&hci_set_event_mask());
        // info!("Filtering unwanted le events");
        self.write_cmd(&hci_le_set_event_mask());

        // info!("Set scan params: allow all");
        self.write_cmd(&hci_le_set_scan_params(false));
        info!("Ble Scanner Started");
    }

    /// Writes the given HCI Command packet to the Bluetooth controller. This
    /// waits for the HCI Command Complete Event packet from the controller
    /// to ensure it was fully processed with no errors.
    fn write_cmd(&mut self, packet: &[u8]) {
        let opcode_lsb = packet[1];
        let opcode_msb = packet[2];

        self.ble.write_all(packet).unwrap();
        self.ble.flush().unwrap();

        // Wait for a command complete event with the opcode we just sent
        let mut buffer = [0u8; 256];
        loop {
            let len = self.ble.get_next(&mut buffer).unwrap();

            if len != 7 || buffer[0] != PACKET_TYPE_EVENT || buffer[1] != EVENT_COMMAND_COMPLETE {
                continue;
            }

            // The 2-byte opcode should match the opcode for the command
            // that was just written. If it doesn't, then some other command
            // was issued without waiting for this event, which shouldn't
            // happen since that's what we're doing now.
            if buffer[4] != opcode_lsb || buffer[5] != opcode_msb {
                panic!("Unhandled Command Complete Event: {:02X?}", &buffer)
            }

            // The last byte is the exit code, with 0 indicating success
            if buffer[6] != 0x00 {
                panic!(
                    "HCI command failed. Error code: {}. Command: {:02X?}",
                    buffer[6], packet
                );
            } else {
                break;
            }
        }
    }

    /// Waits for a Ble data packet to come in, but only until `scan_end_time`,
    /// Returns None if no Ble data was received before the end time.
    async fn wait_for_ble_event(&mut self, scan_end_time: Instant) {
        let mut buffer = [0u8; 256];

        while Instant::now() < scan_end_time {
            embassy_futures::yield_now().await;

            match self.ble.get_next(&mut buffer) {
                Err(e) => {
                    warn!("Read error: {:?}", e);
                }
                Ok(0) => {}
                Ok(len) => {
                    self.packets += 1;
                    self.packets_size += len;
                }
            }
        }
    }

    pub async fn scan_until(&mut self, scan_end_time: Instant) {
        self.write_cmd(&hci_le_set_scan_enable(true, false));
        info!("scan started");
        self.wait_for_ble_event(scan_end_time).await;
        info!("scan stopped");
        self.write_cmd(&hci_le_set_scan_enable(false, false));
    }
}

/// Resets the bluetooth controller to its default state.
fn hci_reset() -> [u8; PACKET_HEADER_LENGTH] {
    hci_cmd_packet::<0>(OPCODE_RESET, [])
}

/// Filters out all events except the LE Meta Event.
fn hci_set_event_mask() -> [u8; 8 + PACKET_HEADER_LENGTH] {
    hci_cmd_packet::<8>(
        OPCODE_SET_EVENT_MASK,
        [
            // Disable all events
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, // Except the LE Meta Event
        ],
    )
}

/// Filters out all LE Meta Events except the LE Advertising Report Event.
fn hci_le_set_event_mask() -> [u8; 8 + PACKET_HEADER_LENGTH] {
    hci_cmd_packet::<8>(
        OPCODE_LE_SET_EVENT_MASK,
        [
            // Disable all events
            0x02, // Except the LE Advertising Report Event
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ],
    )
}

/// Sets the parameters for the LE scan. This will perform a passive scan for
/// the configured interval and window. It can optionally filter out unwanted
/// addresses.
fn hci_le_set_scan_params(filter: bool) -> [u8; 7 + PACKET_HEADER_LENGTH] {
    let filter_param = if filter {
        // Only report events from addresses that have been added to the list
        // via hci_le_add_to_white_list.
        SCAN_PARAM_FILTER_ALLOW_LISTED
    } else {
        // Do not filter, allow all
        SCAN_PARAM_FILTER_ALLOW_ALL
    };

    hci_cmd_packet::<7>(
        OPCODE_SET_SCAN_PARAMS,
        [
            0x00, // Scan type: passive
            SCAN_PARAM_SCAN_INTERVAL as u8,
            (SCAN_PARAM_SCAN_INTERVAL >> 8) as u8,
            SCAN_PARAM_SCAN_WINDOW as u8,
            (SCAN_PARAM_SCAN_WINDOW >> 8) as u8,
            0x00, // Own address type: public
            filter_param,
        ],
    )
}

/// Allows the BLE address to be reported in LE scans if the scan is
/// set with the filter enabled.
// fn hci_le_add_to_white_list(pkt: BlePacket) -> [u8; 7 + PACKET_HEADER_LENGTH] {
//     hci_cmd_packet::<7>(OPCODE_ADD_TO_WHITELIST, *pkt.address())
// }

/// Enables or disables the LE scan. Optionally duplicate addresses can be
/// filtered out.
fn hci_le_set_scan_enable(enable: bool, filter_duplicates: bool) -> [u8; 2 + PACKET_HEADER_LENGTH] {
    let enable_param = if enable { 1 } else { 0 };
    let duplicates_param = if filter_duplicates { 1 } else { 0 };

    hci_cmd_packet::<2>(OPCODE_SET_SCAN_ENABLE, [enable_param, duplicates_param])
}

/// Constructs an HCI Command packet. The packet is a 1-byte packet type (0x01),
/// a 2-byte opcode little-endian encoded, 1-byte describing the length of the
/// data in bytes, followed by that data.
/// The data is different for each command opcode. `N` is the length of the
/// data.
fn hci_cmd_packet<const N: usize>(opcode: u16, params: [u8; N]) -> [u8; N + PACKET_HEADER_LENGTH] {
    let mut packet = [0u8; N + PACKET_HEADER_LENGTH];
    packet[0] = PACKET_TYPE_COMMAND;
    packet[1] = opcode as u8;
    packet[2] = (opcode >> 8) as u8;
    packet[3] = N as u8;
    packet[4..].copy_from_slice(&params);
    packet
}
