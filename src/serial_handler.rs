use heapless::Vec;
use opilio_lib::{
    error::Error, Cmd, Data, Response, Result, Stats, MAX_SERIAL_DATA_SIZE, OTW,
};
use postcard::to_vec;
use stm32f1xx_hal::flash;
use usb_device::{bus::UsbBus, prelude::UsbDevice};
use usbd_serial::SerialPort;

use crate::{controller::Controller, tacho::TachoReader, Config, FlashOps};

#[inline(always)]
pub fn usb_poll<B: UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    config: &mut Config,
    flash: &mut flash::Parts,
    controller: &mut Controller,
    tacho: &mut TachoReader,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }
    if let Err(e) = process_command(serial, config, tacho, controller, flash) {
        defmt::trace!("usb_poll error: {}", e);
    }
}

#[inline(always)]
fn process_command<B: UsbBus>(
    serial: &mut SerialPort<'static, B>,
    config: &mut Config,
    tacho: &mut TachoReader,
    controller: &mut Controller,
    flash: &mut flash::Parts,
) -> Result<()> {
    let mut buf = [0u8; MAX_SERIAL_DATA_SIZE];
    let count = serial.read(&mut buf).map_err(|_| Error::SerialRead)?;

    if count == 0 {
        return Ok(());
    }

    defmt::debug!("bytes read: {}", count);
    defmt::trace!("BUF {:?}", buf);
    if count == 64 {
        if let Ok(count) = serial.read(&mut buf[count..]) {
            defmt::debug!("bytes second read: {}", count);
            defmt::trace!("BUF {:?}", buf);
        }
    }
    let otw_in = OTW::from_bytes(&buf)?;
    defmt::debug!("Received {:?}", otw_in.cmd());
    match otw_in.cmd() {
        Cmd::SetConfig => {
            if let Data::Config(new_config) = otw_in.data() {
                *config = new_config;
                let bytes: Vec<u8, 3> = to_vec(&Response::Ok)?;
                serial
                    .write(bytes.as_ref())
                    .map_err(|_| Error::SerialWrite)?;
            }
        }
        Cmd::GetConfig => {
            let bytes = OTW::new(Cmd::Config, Data::Config(config.clone()))?
                .to_vec()?;
            serial
                .write(bytes.as_ref())
                .map_err(|_| Error::SerialWrite)?;
        }
        Cmd::GetStats => {
            let (pump1_rpm, fan1_rpm, fan2_rpm, fan3_rpm) = tacho.rpm_data();
            let stats = Stats {
                pump1_rpm,
                fan1_rpm,
                fan2_rpm,
                fan3_rpm,
                liquid_temp: controller.get_liquid_temp(),
                ambient_temp: controller.get_ambient_temp(),
            };
            let otw = OTW::new(Cmd::Stats, Data::Stats(stats))?.to_vec()?;
            serial.write(otw.as_ref()).map_err(|_| Error::SerialWrite)?;
        }
        Cmd::SaveConfig => {
            if let Err(e) = config.save_to_flash(flash) {
                let otw =
                    OTW::new(Cmd::Result, Data::Result(Response::Error(e)))?
                        .to_vec()?;
                serial.write(otw.as_ref()).map_err(|_| Error::SerialWrite)?;
                return Err(e);
            } else {
                let otw = OTW::new(Cmd::Result, Data::Result(Response::Ok))?
                    .to_vec()?;
                serial.write(otw.as_ref()).map_err(|_| Error::SerialWrite)?;
            }
        }
        Cmd::SetStandby => {
            if let Data::U64(sleep_after) = otw_in.data() {
                config.sleep_after_ms = sleep_after;
                let otw = OTW::new(Cmd::Result, Data::Result(Response::Ok))?
                    .to_vec()?;
                serial.write(otw.as_ref()).map_err(|_| Error::SerialWrite)?;
            }
        }
        Cmd::Stats | Cmd::Config | Cmd::Result => (),
    };

    Ok(())
}
