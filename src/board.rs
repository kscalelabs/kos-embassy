use embassy_stm32::mode::Async;
use embassy_stm32::spi::SpiSlave;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{Config as UsartConfig, Uart};
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, Output, Pull, Speed},
    i2c, peripherals, rcc, spi, usart, Config,
};

// ── IRQ table ─────────────────────────────────────────────
bind_interrupts!(pub struct Irqs {
    //USART1 => usart::BufferedInterruptHandler<peripherals::USART1>;
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    I2C2   => i2c::EventInterruptHandler<peripherals::I2C2>,
              i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

static mut FEETECH_DMA_BUF: [u8; 2048] = [0; 2048];

// ── Board struct ──────────────────────────────────────────
pub struct Board {
    //pub feetech_uart: BufferedUart<'static>,
    //pub feetech_uart: Uart<'static, Async>,
    pub feetech_tx: usart::UartTx<'static, Async>,
    pub feetech_rx: usart::RingBufferedUartRx<'static>,
    pub i2c1: i2c::I2c<'static, Async>, // DMA
    pub spi1: SpiSlave<'static, Async>, // DMA
    pub data_ready_spi: Output<'static>,
}

impl Board {
    pub fn init() -> Self {
        // Configure for maximum performance
        let mut config = Config::default();

        // Enable HSI and configure PLL for 64MHz
        config.rcc.hsi = Some(rcc::Hsi {
            sys_div: rcc::HsiSysDiv::DIV1, // No division for maximum speed
        });
        config.rcc.pll = Some(rcc::Pll {
            source: rcc::PllSource::HSI,    // Use HSI as PLL source
            prediv: rcc::PllPreDiv::DIV2,   // 16MHz / 2 = 8MHz
            mul: rcc::PllMul::MUL16,        // 8MHz * 16 = 128MHz
            divp: None,                     // Not used
            divq: None,                     // Not used
            divr: Some(rcc::PllRDiv::DIV2), // 128MHz / 2 = 64MHz
        });
        config.rcc.sys = rcc::Sysclk::PLL1_R; // Use PLL as system clock

        //config.rcc.hsi = Some(rcc::Hsi { sys_div: rcc::HsiSysDiv::DIV1 });
        // Optional: Configure AHB/APB prescalers
        //config.rcc.ahb_pre = rcc::AhbPrescaler::DIV1;   // AHB = 64MHz
        //config.rcc.apb_pre = rcc::ApbPrescaler::DIV1;   // APB = 64MHz
        let p = embassy_stm32::init(config);

        let data_ready_spi = Output::new(p.PA9, Level::Low, Speed::Low);

        let mut us_cfg = UsartConfig::default();
        us_cfg.baudrate = 1_000_000;
        us_cfg.rx_pull = Pull::Up;


        //let  uart = Uart::new(p.USART1, p.PB7, p.PB6, Irqs, p.DMA1_CH2, p.DMA1_CH3, us_cfg).unwrap();
        let uart = Uart::new(p.USART1, p.PC5, p.PC4, Irqs, p.DMA1_CH2, p.DMA1_CH3, us_cfg).unwrap();
        let (tx, rx) = uart.split();
        // DMA-circular RX driver

        #[allow(static_mut_refs)]
        let rx = rx.into_ring_buffered(unsafe { &mut FEETECH_DMA_BUF });

        // I²C1  (DMA CH4 TX, CH5 RX)
        let mut i2c_cfg = i2c::Config::default();
        i2c_cfg.sda_pullup = false;
        i2c_cfg.scl_pullup = false;

        let i2c1 = i2c::I2c::new(
            p.I2C2,
            p.PB10,
            p.PB11,
            Irqs,
            p.DMA1_CH7,
            p.DMA1_CH6,
            Hertz(100_000),
            i2c_cfg,
        );

        // SPI1  (DMA CH3 TX, CH2 RX)
        let mut spi_cfg = spi::ConfigSlave::default();
        spi_cfg.mode = spi::MODE_0;
        //spi_cfg.slave = true;
        let spi1 = SpiSlave::new_hardware_cs(
            p.SPI1,
            p.PA5, p.PA7, p.PA6, p.PA4, // SCK, MOSI, MISO, NSS
            p.DMA1_CH5,          // TX
            p.DMA1_CH4,          // RX
            spi_cfg,
        );

        /*let spi1 = SpiSlave::new_hardware_cs(
            p.SPI1, p.PD8, p.PA7, p.PA6, p.PA4,      // SCK, MOSI, MISO, NSS
            p.DMA1_CH5, // TX
            p.DMA1_CH4, // RX
            spi_cfg,
        );*/

        Self {
            feetech_tx: tx,
            feetech_rx: rx,
            i2c1,
            spi1,
            data_ready_spi,
        }
    }
}
