use core::fmt::Write;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    instr, Common, Config, InterruptHandler, Irq, Pio, PioPin, ShiftDirection, StateMachine,
};
use embassy_time::{with_timeout, Duration};
use fixed::traits::ToFixed;
use heapless;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const SPI_BITS: u32 = 24;
pub const MAX_STRING_SIZE: usize = 40;

fn setup_pio_task<'a>(
    pio: &mut Common<'a, PIO0>,
    sm: &mut StateMachine<'a, PIO0, 0>,
    clk_pin: impl PioPin,
    miso_pin: impl PioPin,
) {
    let prg = pio_proc::pio_file!("src/indicator.pio");

    let mut cfg = Config::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    cfg.clock_divider = 100.to_fixed();
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.threshold = 32;
    cfg.shift_in.direction = ShiftDirection::Left;
    cfg.shift_out.auto_fill = false;

    let clk_pin = pio.make_pio_pin(clk_pin);
    let miso_pin = pio.make_pio_pin(miso_pin);
    cfg.set_in_pins(&[&clk_pin, &miso_pin]);
    sm.set_pin_dirs(embassy_rp::pio::Direction::In, &[&clk_pin, &miso_pin]);
    sm.set_config(&cfg);
}

pub async fn get_bits(
    sm: &mut StateMachine<'static, PIO0, 0>,
    irq: &mut Irq<'static, PIO0, 3>,
    led: &mut Output<'static>,
) -> [bool; SPI_BITS as usize] {
    sm.set_enable(true);
    loop {
        sm.clear_fifos();
        sm.restart();
        //this is safe sine the state machine is on and the origin of the program is set to 0
        unsafe {
            instr::exec_jmp(sm, 0);
        }
        sm.tx().push(SPI_BITS - 1);

        let res = with_timeout(Duration::from_millis(100), irq.wait()).await;
        if let Err(_) = res {
            continue;
        }
        led.toggle();

        let mut num_pulled = 0;
        //instantiate a result array which is SPI_BITS long and boollean
        let mut result = [false; SPI_BITS as usize];
        while num_pulled < SPI_BITS {
            let rx = sm.rx().pull();

            for i in 0..u32::BITS.min(SPI_BITS - num_pulled) {
                result[(SPI_BITS - num_pulled - i - 1) as usize] = (rx & (1 << i)) != 0;
            }
            num_pulled += u32::BITS;
        }
        return result;
    }
}

pub fn pio_init(
    pio: PIO0,
    clk_pin: impl PioPin,
    miso_pin: impl PioPin,
) -> (Irq<'static, PIO0, 3>, StateMachine<'static, PIO0, 0>) {
    let Pio {
        mut common,
        mut sm0,
        irq3,
        ..
    } = Pio::new(pio, Irqs);

    setup_pio_task(&mut common, &mut sm0, clk_pin, miso_pin);

    return (irq3, sm0);
}

pub fn format_bits(bits: [bool; SPI_BITS as usize]) -> heapless::String<MAX_STRING_SIZE> {
    let mut s: heapless::String<MAX_STRING_SIZE> = heapless::String::new();
    //the first 16 bits are the data - convert them to a u16
    let mut value: i32 = 0;
    for i in 0..16 {
        if !bits[i] {
            value += 1 << i;
        }
    }
    let sign = bits[20];
    if !sign {
        value = -value;
    }

    let unit = bits[23];
    if unit {
        let distance = value as f32 / 1000.0;
        write!(s, "{}mm", distance).unwrap();
    } else {
        let distance = value as f32 / 20000.0;
        write!(s, "{}in", distance).unwrap();
    }

    // let distance = distance as f32 / 500.0;

    s
}
