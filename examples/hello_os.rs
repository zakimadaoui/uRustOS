#![no_main]
#![no_std]

use uRustOS::OS::*;
use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::itm::write_fmt;
use cortex_m::peripheral::ITM;
use stm32f1xx_hal::{pac::Peripherals, prelude::*};

static mut G_ITM: Option<ITM> = None;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let _clocks = rcc
        .cfgr
        .sysclk(8.MHz())
        .hclk(8.MHz())
        .pclk1(8.MHz())
        .freeze(&mut flash.acr);

    let cp = cortex_m::Peripherals::take().unwrap();
    let itm = cp.ITM;

    unsafe {
        G_ITM = Some(itm);
    }

    let mut stack1: [u32; 500] = [1; 500];
    let mut thread1 = OSThread::new(&mut stack1, thread_run);
    thread1.schedule();

    let mut stack2: [u32; 500] = [2; 500];
    let mut thread2 = OSThread::new(&mut stack2, thread_run);
    thread2.schedule();

    let mut stack3: [u32; 500] = [2; 500];
    let mut thread3 = OSThread::new(&mut stack3, thread_run);
    thread3.schedule();

    // only call os_run after all the needed threads have been scheduled 
    // os_run will start the OS scheduler
    let status: bool = os_run(8.MHz());

    if status {
        // if the os succeeds to run then this is unreachable code, control will
        // never go back to main
    } else {
        // OS was not able to run since there is no
    }

    // if the os succeeds to run then this is unreachable code, control will never go
    // back to main

    loop { /* dead code */ }
}

fn thread_run() {
    loop {
        //print to itm
        unsafe {
            write_fmt(
                &mut G_ITM.as_mut().unwrap().stim[0],
                format_args!("hello from thread: {}\n", os_get_thread_nbr()),
            );
        }
        for _ in 0..9999 {} //some delay
    }
}
