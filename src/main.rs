#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate alloc;

use bme680::{self, Bme680};
use core::alloc::Layout;
use core::ptr;
use core::time::Duration;
use esp_idf_hal::{delay, i2c};
use esp_idf_sys::*;

const UART_NUM: uart_port_t = uart_port_t_UART_NUM_1;
const ECHO_TEST_TXD: i32 = gpio_num_t_GPIO_NUM_1 as i32;
const ECHO_TEST_RXD: i32 = gpio_num_t_GPIO_NUM_3 as i32;
const ECHO_TEST_RTS: i32 = UART_PIN_NO_CHANGE;
const ECHO_TEST_CTS: i32 = UART_PIN_NO_CHANGE;
const BUF_SIZE: i32 = 1024;

#[global_allocator]
static A: esp_idf_alloc::EspIdfAllocator = esp_idf_alloc::EspIdfAllocator;

extern "C" {
    fn abort() -> !;
}

#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    unsafe {
        abort();
    }
}

#[no_mangle]
pub fn app_main() {
    unsafe {
        rust_blink_and_write();
    }
}

unsafe fn rust_blink_and_write() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    let uart_config = uart_config_t {
        baud_rate: 115200,
        data_bits: uart_word_length_t_UART_DATA_8_BITS,
        parity: uart_parity_t_UART_PARITY_DISABLE,
        stop_bits: uart_stop_bits_t_UART_STOP_BITS_1,
        flow_ctrl: uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
        rx_flow_ctrl_thresh: 0,
        use_ref_tick: false,
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(
        UART_NUM,
        ECHO_TEST_TXD,
        ECHO_TEST_RXD,
        ECHO_TEST_RTS,
        ECHO_TEST_CTS,
    );
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, ptr::null_mut(), 0);

    let sda_pin_config = i2c::PinConfig {
        pin_num: gpio_num_t_GPIO_NUM_21,
        pullup: false,
    };

    let scl_pin_config = i2c::PinConfig {
        pin_num: gpio_num_t_GPIO_NUM_22,
        pullup: false,
    };

    let clk_speed = 100_000;

    let i2c_master =
        i2c::Master::new(i2c::Port::Port0, sda_pin_config, scl_pin_config, clk_speed).unwrap();

    let mut dev = Bme680::init(i2c_master, delay::Ets {}, bme680::I2CAddress::Primary).unwrap();

    let settings = bme680::SettingsBuilder::new()
        .with_humidity_oversampling(bme680::OversamplingSetting::OS2x)
        .with_pressure_oversampling(bme680::OversamplingSetting::OS4x)
        .with_temperature_oversampling(bme680::OversamplingSetting::OS8x)
        .with_temperature_filter(bme680::IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();

    dev.set_sensor_settings(settings).unwrap();

    dev.set_sensor_mode(bme680::PowerMode::ForcedMode).unwrap();

    loop {
        let (data, state) = dev.get_sensor_data().unwrap();

        match state {
            bme680::FieldDataCondition::NewData => {
                let msg = format!("NEW TEMP: {}C.\n", data.temperature_celsius());
                uart_write_bytes(UART_NUM, msg.as_ptr() as *const _, msg.len());

                let msg = format!("NEW HUMIDITY: {}%.\n", data.humidity_percent());
                uart_write_bytes(UART_NUM, msg.as_ptr() as *const _, msg.len());

                let msg = format!("NEW PRESSURE: {}hPa.\n", data.pressure_hpa());
                uart_write_bytes(UART_NUM, msg.as_ptr() as *const _, msg.len());
            }
            bme680::FieldDataCondition::Unchanged => {
                let msg = ".\n";
                uart_write_bytes(UART_NUM, msg.as_ptr() as *const _, msg.len());
            }
        }

        ets_delay_us(1_000_000);
    }
}
