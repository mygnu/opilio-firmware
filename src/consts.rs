pub const MAX_DUTY: u16 = 333;
pub const MIN_DUTY: u16 = 33; // 10% usually when a pwm fan starts to spin
/// 10k resistor measured resistance in Ohms
pub const R_10K: f32 = 9950.0;
/// voltage
pub const V_SUPPLY: f32 = 3.3;
/// B-coefficient of the thermistor (guessed)
pub const B_PARAM: f32 = 3700.0; //3200.0;
/// 0*C in kelvin
pub const ZERO_K_IN_C: f32 = 273.15;

pub const MIN_TEMP: f32 = 20.0;
pub const MAX_TEMP: f32 = 40.0;
// Analog to digital resolution
pub const ADC_RESOLUTION: f32 = 4096.0;

/// start address: 0x08000000
/// used by program: 50 KIB or 51200 bytes
/// we can use the rest of the 64K memory (14kb) for storage
pub const FLASH_START_OFFSET: u32 = 0x0C800;
