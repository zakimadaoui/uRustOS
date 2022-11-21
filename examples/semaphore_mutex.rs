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
    
    
    let mut stack1: [u32; 500] = [1; 500];
    let mut thread1 = OSThread::new(&mut stack1, thread1_run);
    thread1.schedule();
    
    let mut stack2: [u32; 500] = [2; 500];
    let mut thread2 = OSThread::new(&mut stack2, thread2_run);
    thread2.schedule();
    
    let mut stack3: [u32; 500] = [2; 500];
    let mut thread3 = OSThread::new(&mut stack3, thread3_run);
    thread3.schedule();
    
    
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


/* YOU CAN COMMENT THE MUTEX WAIT AND SIGNAL CALLS TO SEE RACE CONDITIONS */

static mut G_COUNTER: u32 = 0;
static mut mutex: Semaphore = Semaphore::new_mutex();


fn thread1_run() {
    unsafe {
        let mut local;
        loop{
            // mutex.wait();
            local = G_COUNTER + 1;
            for _ in 0..50 {}
            G_COUNTER = local;
            write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("1:{}\n",G_COUNTER));
            for _ in 0..50 {}
            // mutex.signal();
            for _ in 0..99 {} //avoid starvation
            
        }
        
    }
}

fn thread2_run() {
    unsafe {
        let mut local;
        loop{
            // mutex.wait();
            local = G_COUNTER + 1;
            for _ in 0..50 {}
            G_COUNTER = local;
            write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("2:{}\n",G_COUNTER));
            for _ in 0..50 {}
            // mutex.signal();
            for _ in 0..99 {} //avoid starvation
            
        }
    }
}

fn thread3_run() {
    unsafe {
        let mut local;
        loop{
            // mutex.wait();
            local = G_COUNTER + 1;
            for _ in 0..50 {}
            G_COUNTER = local;
            write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("3:{}\n",G_COUNTER));
            for _ in 0..50 {}
            // mutex.signal();
            for _ in 0..99 {} //avoid starvation
        }
    }
}

