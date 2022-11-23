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

    // Configure PC13 pin to blink LED
    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let _ = led.set_low(); // Turn off


    let cp = cortex_m::Peripherals::take().unwrap();
    let itm = cp.ITM;

    unsafe{ 
        G_ITM = Some(itm);
    }
    
    let mut stack1: [u32; 500] = [2; 500];
    let mut thread1 = OSThread::new(&mut stack1, producer_thread);
    thread1.schedule();
    
    let mut stack2: [u32; 500] = [2; 500];
    let mut thread2 = OSThread::new(&mut stack2, consumer_thread);
    thread2.schedule();
    
    // only call os_run after all the needed threads have been scheduled 
    // os_run will start the OS scheduler
    let status : bool = os_run(8.MHz());

    if status  {
        // if the os succeeds to run then this is unreachable code, control will 
        // never go back to main
    } else {
        // OS was not able to run since there is no
    }

    // if the os succeeds to run then this is unreachable code, control will never go
    // back to main

    loop {
        /* dead code */
    }
}

static mut G_QUEUE : Queue<u32, 10> = Queue::new(0);
fn producer_thread() {
    unsafe {
        let mut local = 0;
        loop{
            G_QUEUE.produce(local);
            write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("P:{}\n",local));
            local = local+1;
            // for _ in 0..999 {} // some delay for visualization
        }
    }
}

fn consumer_thread() {
    unsafe {
        let mut local;
        loop{
            local = G_QUEUE.consume();
            write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("C:{}\n",local));
            // for _ in 0..999 {} // UNCOMMENT for some delay for visualization
        }
    }
}

