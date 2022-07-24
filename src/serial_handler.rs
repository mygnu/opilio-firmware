use opilio_lib::{
    error::Error, Cmd, Data, DataRef, Response, Result, Stats, OTW,
};

use stm32f1xx_hal::flash;
use usb_device::{bus::UsbBus, prelude::UsbDevice, UsbError};
use usbd_serial::SerialPort;

use crate::{controller::Controller, tacho::TachoReader, Config, FlashOps};

pub struct UsbHandler<B: UsbBus + 'static> {
    pub usb_dev: UsbDevice<'static, B>,
    pub serial: SerialPort<'static, B>,
}

impl<B: UsbBus + 'static> UsbHandler<B> {
    pub fn new(
        usb_dev: UsbDevice<'static, B>,
        serial: SerialPort<'static, B>,
    ) -> Self {
        Self { usb_dev, serial }
    }

    pub fn poll(
        &mut self,
        config: &mut Config,
        flash: &mut flash::Parts,
        controller: &mut Controller,
        tacho: &mut TachoReader,
    ) {
        if !self.usb_dev.poll(&mut [&mut self.serial]) {
            return;
        }
        if let Err(e) = self.process_command(tacho, config, controller, flash) {
            defmt::trace!("usb_poll error: {}", e);
        }
    }

    fn process_command(
        &mut self,
        tacho: &mut TachoReader,
        config: &mut Config,
        controller: &mut Controller,
        flash: &mut flash::Parts,
    ) -> Result<()> {
        let mut buf = [0u8; 128];
        let mut cursor =
            self.serial.read(&mut buf).map_err(|_| Error::SerialRead)?;

        if cursor == 0 {
            return Ok(());
        }
        defmt::debug!("bytes read: {}", cursor);
        defmt::trace!("BUF {=[?]}", buf);
        loop {
            match self.serial.read(&mut buf[cursor..]) {
                Ok(count) => {
                    defmt::debug!("count: {}", count);
                    if count == 0 {
                        break;
                    }
                    cursor += count
                }
                Err(e) => match e {
                    UsbError::WouldBlock => break,
                    _ => return Err(Error::SerialRead),
                },
            }
        }

        let otw_in = OTW::from_bytes(&buf)?;
        defmt::debug!("Received {:?}", otw_in.cmd);
        match otw_in.cmd {
            Cmd::UploadSetting => {
                if let Data::Setting(setting) = otw_in.data {
                    config.set(setting);
                    self.serial
                        .write(OTW::serialised_ok())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Cmd::GetConfig => {
                let bytes =
                    OTW::serialised_vec(Cmd::Config, DataRef::Config(&config))?;
                let total = (&bytes).len();
                let mut sent = 0;
                while sent < total {
                    sent += self
                        .serial
                        .write(bytes.as_ref())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Cmd::GetStats => {
                let (pump1_rpm, fan1_rpm, fan2_rpm, fan3_rpm) =
                    tacho.rpm_data();
                let stats = Stats {
                    pump1_rpm,
                    fan1_rpm,
                    fan2_rpm,
                    fan3_rpm,
                    liquid_temp: controller.get_liquid_temp(),
                    ambient_temp: controller.get_ambient_temp(),
                };
                let otw =
                    OTW::serialised_vec(Cmd::Stats, DataRef::Stats(&stats))?;
                self.serial
                    .write(otw.as_ref())
                    .map_err(|_| Error::SerialWrite)?;
            }
            Cmd::SaveConfig => {
                if let Err(e) = config.save_to_flash(flash) {
                    let otw = OTW::serialised_vec(
                        Cmd::Result,
                        DataRef::Result(&Response::Error(e)),
                    )?;
                    self.serial
                        .write(otw.as_ref())
                        .map_err(|_| Error::SerialWrite)?;
                    return Err(e);
                } else {
                    self.serial
                        .write(OTW::serialised_ok())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Cmd::UploadGeneral => {
                if let Data::General(general) = otw_in.data {
                    config.general = general;

                    self.serial
                        .write(OTW::serialised_ok())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Cmd::Stats | Cmd::Config | Cmd::Result => (),
        };

        Ok(())
    }
}
