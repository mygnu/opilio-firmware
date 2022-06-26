use heapless::Vec;
use postcard::{from_bytes, to_vec};
use shared::{Command, Response};
use stm32f1xx_hal::flash;
use usb_device::{bus::UsbBus, prelude::UsbDevice};
use usbd_serial::SerialPort;

use crate::{controller::Controller, tacho::TachoReader, Configs, FlashOps};

#[inline(always)]
pub fn usb_poll<B: UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    configs: &mut Configs,
    flash: &mut flash::Parts,
    controller: &mut Controller,
    tacho: &mut TachoReader,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }
    if let Err(e) = process_command(serial, configs, tacho, controller, flash) {
        defmt::error!("{}", e);
    }
}

#[inline(always)]
fn process_command<B: UsbBus>(
    serial: &mut SerialPort<'static, B>,
    configs: &mut Configs,
    tacho: &mut TachoReader,
    controller: &mut Controller,
    flash: &mut flash::Parts,
) -> shared::Result<()> {
    let mut buf = [0u8; 48];
    let count = serial
        .read(&mut buf)
        .map_err(|_| shared::Error::SerialRead)?;

    if count == 0 {
        return Ok(());
    }

    defmt::trace!("buf {:?}", buf);
    let command = from_bytes::<Command>(&buf[0..1])?;
    match command {
        Command::SetConfig => {
            if let Ok(new_config) = from_bytes(&buf[2..]) {
                configs.set(new_config);
                let bytes: Vec<u8, 3> = to_vec(&Response::Ok)?;
                defmt::info!("Ok: {}", bytes);
                serial
                    .write(bytes.as_ref())
                    .map_err(|_| shared::Error::SerialWrite)?;
            }
        }
        Command::GetConfig => {
            if let Ok(id) = from_bytes(&buf[2..]) {
                defmt::debug!("id {:?}", id);
                if let Some(config) = configs.get(id) {
                    defmt::debug!("{:?}", config);
                    let bytes = config.to_vec()?;
                    defmt::debug!("config bytes {}", bytes);
                    serial
                        .write(bytes.as_ref())
                        .map_err(|_| shared::Error::SerialWrite)?;
                }
            }
        }
        Command::GetRpm => {
            let rpm_data = tacho.rpm_data();
            let bytes = rpm_data.to_vec()?;
            defmt::debug!("rpm bytes {}", bytes);
            serial
                .write(bytes.as_ref())
                .map_err(|_| shared::Error::SerialWrite)?;
        }
        Command::GetTemp => {
            let temp = controller.get_temp();
            let bytes: Vec<u8, 3> = to_vec(&temp)?;
            serial
                .write(bytes.as_ref())
                .map_err(|_| shared::Error::SerialWrite)?;
        }
        Command::SaveConfig => {
            configs.save_to_flash(flash)?;
        }
    };

    Ok(())
}
