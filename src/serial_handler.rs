use defmt::Format;
use heapless::Vec;
use postcard::{from_bytes, to_vec};
use serde::{Deserialize, Serialize};
use stm32f1xx_hal::flash;
use usb_device::{bus::UsbBus, prelude::UsbDevice};
use usbd_serial::SerialPort;

use crate::Configs;

pub fn usb_poll<B: UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    configs: &mut Configs,
    flash: &mut flash::Parts,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 32];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // debug!("{:?}", buf);

            if let Ok(command) = from_bytes::<Command>(&buf[0..1]) {
                if command == Command::SetConfig {
                    if let Ok(new_config) = from_bytes(&buf[2..]) {
                        configs.set(new_config);
                        let ok: Vec<u8, 3> = to_vec(&Response::Ok).unwrap();
                        serial.write(&ok).ok();
                        return;
                    }
                }

                if command == Command::GetConfig {
                    if let Ok(id) = from_bytes(&buf[1..]) {
                        if let Some(config) = configs.get(id) {
                            let config: Vec<u8, 32> = to_vec(&config).unwrap();
                            serial.write(&config).ok();
                            return;
                        }
                    }
                }
                if command == Command::SaveConfig {
                    configs.save_to_flash(flash);
                }
            }

            let error: Vec<u8, 3> = to_vec(&Response::Error).unwrap();
            serial.write(&error).ok();
        }
        _ => {}
    }
}

#[derive(Format, Serialize, Deserialize, PartialEq)]
enum Response {
    Ok,
    Error,
}

#[derive(Format, Serialize, Deserialize, PartialEq)]
enum Command {
    SetConfig = 1,
    GetConfig = 2,
    SaveConfig = 3,
    GetTemp = 10,
    GetRpm = 11,
}

#[derive(Serialize, Deserialize)]
struct SerialData {
    command: Command,
    value: Vec<u8, 32>,
}
