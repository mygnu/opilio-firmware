use opilio_lib::{
    error::Error, Data, DataRef, Msg, Response, Result, Stats, OTW,
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

    #[inline]
    fn process_command(
        &mut self,
        tacho: &mut TachoReader,
        config: &mut Config,
        controller: &mut Controller,
        flash: &mut flash::Parts,
    ) -> Result<()> {
        let mut buf = [0u8; 256];
        let mut bytes_read =
            self.serial.read(&mut buf).map_err(|_| Error::SerialRead)?;

        defmt::debug!("bytes read: {}", bytes_read);
        defmt::trace!("BUF {=[?]}", buf);

        loop {
            match self.serial.read(&mut buf[bytes_read..]) {
                Ok(count) => {
                    bytes_read += count;
                    defmt::trace!("BUF {=[?]}", buf);
                    defmt::trace!("bytes read: {}", bytes_read);
                }
                Err(e) => match e {
                    UsbError::WouldBlock => break,
                    _ => return Err(Error::SerialRead),
                },
            }
        }

        // we can't serialise a message with less than 2 bytes
        if bytes_read < 2 {
            return Err(Error::Deserialize);
        }

        let otw_in = OTW::from_bytes(&buf)?;
        defmt::info!("Received {:?}", otw_in);
        match otw_in.msg {
            Msg::GetConfig => {
                let bytes =
                    OTW::serialised_vec(Msg::Config, DataRef::Config(&config))?;
                let total = (&bytes).len();
                let mut sent = 0;
                while sent < total {
                    sent += self
                        .serial
                        .write(bytes.as_ref())
                        .map_err(|_| Error::SerialWrite)?;
                }
                defmt::info!("Sent {} bytes", sent);
            }
            Msg::GetStats => {
                let (pump1_rpm, fan1_rpm, fan2_rpm, fan3_rpm) =
                    tacho.rpm_data();
                let stats = Stats {
                    pump1_rpm,
                    fan1_rpm,
                    fan2_rpm,
                    fan3_rpm,
                    coolant_temp: controller.get_coolant_temp(),
                    ambient_temp: controller.get_ambient_temp(),
                    coolant_out_temp: controller.get_coolant_out_temp(),
                };
                let otw =
                    OTW::serialised_vec(Msg::Stats, DataRef::Stats(&stats))?;
                self.serial
                    .write(otw.as_ref())
                    .map_err(|_| Error::SerialWrite)?;
            }
            Msg::SaveConfig => {
                if let Err(e) = config.save_to_flash(flash) {
                    let otw = OTW::serialised_vec(
                        Msg::Result,
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
            Msg::Ping => {
                if let Data::Empty = otw_in.data {
                    let otw = OTW::serialised_vec(
                        Msg::Pong,
                        DataRef::Pong(&config.general.sleep_after),
                    )?;

                    self.serial
                        .write(otw.as_ref())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Msg::UploadConfig => {
                if let Data::Config(new_config) = otw_in.data {
                    *config = new_config;

                    self.serial
                        .write(OTW::serialised_ok())
                        .map_err(|_| Error::SerialWrite)?;
                }
            }
            Msg::Reload => {
                *config = Config::from_flash(flash)?;
                self.serial
                    .write(OTW::serialised_ok())
                    .map_err(|_| Error::SerialWrite)?;
            }
            Msg::Stats | Msg::Config | Msg::Result | Msg::Pong => (),
        };

        Ok(())
    }
}
