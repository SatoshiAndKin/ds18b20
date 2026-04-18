#![no_std]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use one_wire_bus::{self, Address, OneWire, OneWireError, OneWireResult};

pub const FAMILY_CODE: u8 = 0x28;

pub mod commands;
mod resolution;

use one_wire_bus::crc::check_crc8;
pub use resolution::Resolution;

#[derive(Debug)]
pub struct SensorData {
    pub temperature: f32,
    pub resolution: Resolution,
    pub alarm_temp_low: i8,
    pub alarm_temp_high: i8,
}

pub struct Ds18b20 {
    address: Address,
}

impl Ds18b20 {
    pub fn new<E>(address: Address) -> OneWireResult<Ds18b20, E> {
        if address.family_code() == FAMILY_CODE {
            Ok(Ds18b20 { address })
        } else {
            Err(OneWireError::FamilyCodeMismatch)
        }
    }

    pub fn address(&self) -> &Address {
        &self.address
    }

    pub fn start_temp_measurement<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E> + OutputPin<Error = E>,
    {
        onewire.send_command(commands::CONVERT_TEMP, Some(&self.address), delay)?;
        Ok(())
    }

    pub fn read_data<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<SensorData, E>
    where
        T: InputPin<Error = E> + OutputPin<Error = E>,
    {
        read_data(&self.address, onewire, delay)
    }

    pub fn set_config<T, E>(
        &self,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: Resolution,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E> + OutputPin<Error = E>,
    {
        onewire.send_command(commands::WRITE_SCRATCHPAD, Some(&self.address), delay)?;
        onewire.write_byte(alarm_temp_high.to_ne_bytes()[0], delay)?;
        onewire.write_byte(alarm_temp_low.to_ne_bytes()[0], delay)?;
        onewire.write_byte(resolution.to_config_register(), delay)?;
        Ok(())
    }

    pub fn save_to_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E> + OutputPin<Error = E>,
    {
        save_to_eeprom(Some(&self.address), onewire, delay)
    }

    pub fn recall_from_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayNs,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E> + OutputPin<Error = E>,
    {
        recall_from_eeprom(Some(&self.address), onewire, delay)
    }
}

pub fn start_simultaneous_temp_measurement<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E> + OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.skip_address(delay)?;
    onewire.write_byte(commands::CONVERT_TEMP, delay)?;
    Ok(())
}

pub fn read_scratchpad<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<[u8; 9], E>
where
    T: InputPin<Error = E> + OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.match_address(address, delay)?;
    onewire.write_byte(commands::READ_SCRATCHPAD, delay)?;
    let mut scratchpad = [0; 9];
    onewire.read_bytes(&mut scratchpad, delay)?;
    check_crc8(&scratchpad)?;
    Ok(scratchpad)
}

fn read_data<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<SensorData, E>
where
    T: InputPin<Error = E> + OutputPin<Error = E>,
{
    let scratchpad = read_scratchpad(address, onewire, delay)?;

    let resolution = if let Some(resolution) = Resolution::from_config_register(scratchpad[4]) {
        resolution
    } else {
        return Err(OneWireError::CrcMismatch);
    };
    let raw_temp = u16::from_le_bytes([scratchpad[0], scratchpad[1]]);
    let temperature = match resolution {
        Resolution::Bits12 => (raw_temp as f32) / 16.0,
        Resolution::Bits11 => (raw_temp as f32) / 8.0,
        Resolution::Bits10 => (raw_temp as f32) / 4.0,
        Resolution::Bits9 => (raw_temp as f32) / 2.0,
    };
    Ok(SensorData {
        temperature,
        resolution,
        alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
        alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
    })
}

fn recall_from_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E> + OutputPin<Error = E>,
{
    onewire.send_command(commands::RECALL_EEPROM, address, delay)?;

    let max_retries = (10000 / one_wire_bus::READ_SLOT_DURATION_MICROS as u32) + 1;
    for _ in 0..max_retries {
        if onewire.read_bit(delay)? {
            return Ok(());
        }
    }
    Err(OneWireError::Timeout)
}

fn save_to_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayNs,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E> + OutputPin<Error = E>,
{
    onewire.send_command(commands::COPY_SCRATCHPAD, address, delay)?;
    delay.delay_ms(10);
    Ok(())
}
