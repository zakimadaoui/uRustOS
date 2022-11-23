
#![no_main]
#![no_std]


use panic_halt as _;
use stm32f1xx_hal::{
    gpio::{gpiob, Output, PushPull, Pin},
    pac::{Peripherals, gpioa::CRH},
    prelude::*,
}; 

use cortex_m_rt::entry;


use uRustOS::OS::*;
use cortex_m::peripheral::ITM;

// A type definition for the GPIO pin to be used for our LED

// Make LED pins globally available
static mut G_LED1: Option<gpiob::PB7<Output<PushPull>>> = None;
static mut G_LED2: Option<gpiob::PB8<Output<PushPull>>> = None;
static mut G_LED3: Option<gpiob::PB9<Output<PushPull>>> = None;
static mut G_ITM: Option<ITM> = None;
static mut MUTEX: u32 = 1;

#[entry]
fn main() -> ! {

    let mut sem: u32 = 1;

    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(8.MHz())
        .hclk(8.MHz())
        .pclk1(8.MHz())
        .freeze(&mut flash.acr);

    // Configure PC13 pin to blink LED
    let mut gpiob = dp.GPIOB.split();
    let mut led1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    let mut led2 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    let mut led3 = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);


    unsafe{ 
        G_LED1 = Some(led1);
        G_LED2 = Some(led2);
        G_LED3 = Some(led3);
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
        // wfi(); // this causes issues with serial debug, do not enable for now !
        // iprintln!(&mut itm.stim[0], "Hello, world!");
    }
}



fn thread1_run() {
    let mut led ;
    unsafe {led = G_LED1.as_mut().unwrap();}
    loop {
        led.toggle();
        os_sleep(100);
   }
}

fn thread2_run() {
    let mut led ;
    unsafe {led = G_LED2.as_mut().unwrap();}
    loop {
        led.toggle();
        os_sleep(250);
   }
}
fn thread3_run() {
    let mut led ;
    unsafe {led = G_LED3.as_mut().unwrap();}
    loop {
        led.toggle();
        os_sleep(500);
   }
}
