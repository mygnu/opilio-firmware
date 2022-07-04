use heapless::Vec;
use postcard::to_vec;
use shared::{
    error::Error, Cmd, Data, Response, Result, Stats, MAX_SERIAL_DATA_SIZE, OTW,
};
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
        defmt::trace!("usb_poll error: {}", e);
    }
}

#[inline(always)]
fn process_command<B: UsbBus>(
    serial: &mut SerialPort<'static, B>,
    configs: &mut Configs,
    tacho: &mut TachoReader,
    controller: &mut Controller,
    flash: &mut flash::Parts,
) -> Result<()> {
    let mut buf = [0u8; MAX_SERIAL_DATA_SIZE];
    let count = serial.read(&mut buf).map_err(|_| Error::SerialRead)?;

    if count == 0 {
        return Ok(());
    }

    defmt::debug!("Input buf {:?}", buf);
    let otw_in = OTW::from_bytes(&buf)?;
    defmt::info!("Received {:?}", otw_in);
    match otw_in.cmd() {
        Cmd::SetConfig => {
            if let Data::Config(new_config) = otw_in.data() {
                configs.set(new_config);
                let bytes: Vec<u8, 3> = to_vec(&Response::Ok)?;
                defmt::info!("Ok: {}", bytes);
                serial
                    .write(bytes.as_ref())
                    .map_err(|_| Error::SerialWrite)?;
            }
        }
        Cmd::GetConfig => {
            if let Data::FanId(id) = otw_in.data() {
                defmt::debug!("id {:?}", id);
                if let Some(config) = configs.get(id) {
                    let bytes =
                        OTW::new(Cmd::Config, Data::Config(config.clone()))?
                            .to_vec()?;
                    serial
                        .write(bytes.as_ref())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
        }
        Cmd::GetStats => {
            let (rpm1, rpm2, rpm3, rpm4) = tacho.rpm_data();
            let stats = Stats {
                rpm1,
                rpm2,
                rpm3,
                rpm4,
                temp1: controller.get_temp(),
            };
            let otw = OTW::new(Cmd::Stats, Data::Stats(stats))?.to_vec()?;
            defmt::debug!("stats bytes {}", otw);
            serial.write(otw.as_ref()).map_err(|_| Error::SerialWrite)?;
        }

        Cmd::SaveConfig => {
            configs.save_to_flash(flash)?;
        }
        Cmd::Stats | Cmd::Config | Cmd::Result => todo!(),
    };

    Ok(())
}
