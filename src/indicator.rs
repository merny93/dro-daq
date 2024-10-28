use embassy_rp::bind_interrupts;
use embassy_rp::gpio::Output;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Common, Config, InterruptHandler, Irq, Pio, ShiftConfig, ShiftDirection, StateMachine,
};
use fixed::traits::ToFixed;
use fixed_macro::types::U56F8;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

fn setup_pio_task_sm2<'a>(pio: &mut Common<'a, PIO0>, sm: &mut StateMachine<'a, PIO0, 2>) {
    // Setup sm2

    // // Repeatedly trigger IRQ 3
    // let prg = pio_proc::pio_asm!(
    //     ".origin 0",
    //     ".wrap_target",
    //     "set x,30",
    //     "delay:",
    //     "jmp x-- delay [15]",
    //     "set y, 0x02",
    //     "in y, 2",
    //     // "irq 3 [15]",
    //     ".wrap",
    // );
    // let mut cfg = Config::default();
    // cfg.use_program(&pio.load_program(&prg.program), &[]);
    // cfg.clock_divider = (U56F8!(125_000_000) / 2000).to_fixed();
    // cfg.shift_out = ShiftConfig{
    //     auto_fill: true,
    //     threshold: 4,
    //     direction: ShiftDirection::Right,
    // };
    // sm.set_config(&cfg);
    // Read 0b10101 repeatedly until ISR is full
    let prg = pio_proc::pio_asm!(
        //
        ".origin 8",
        "set x, 0x15",
        ".wrap_target",
        "in x, 5 [31]",
        ".wrap",
    );

    let mut cfg = Config::default();
    cfg.use_program(&pio.load_program(&prg.program), &[]);
    cfg.clock_divider = (U56F8!(125_000_000) / 2000).to_fixed();
    cfg.shift_in.auto_fill = true;
    cfg.shift_in.direction = ShiftDirection::Right;
    sm.set_config(&cfg);
}

#[embassy_executor::task]
pub async fn pio_task_sm2(
    mut irq: Irq<'static, PIO0, 3>,
    mut sm: StateMachine<'static, PIO0, 2>,
    mut led: Output<'static>,
) {
    sm.set_enable(true);
    loop {
        // irq.wait().await;
        let rx = sm.rx().wait_pull().await;
        led.toggle();
    }
}

pub fn pio_init(pio: PIO0) -> (Irq<'static, PIO0, 3>, StateMachine<'static, PIO0, 2>) {
    let Pio {
        mut common,
        mut sm2,
        irq3,
        ..
    } = Pio::new(pio, Irqs);

    setup_pio_task_sm2(&mut common, &mut sm2);

    return (irq3, sm2);
}
