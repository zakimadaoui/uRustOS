
// TODO: THIS EXAMPLE SHOULD BLINK MULTIPLE LEDS AT SAME TIME AND SHOWS HOW TO USE OS SLEEP



// #![no_main]
// #![no_std]


// use panic_halt as _;
// use stm32f1xx_hal::{
//     gpio::{gpioc, Output, PushPull},
//     pac::{Peripherals},
//     prelude::*,
// }; 

// use cortex_m_rt::entry;


// use uRustOS::OS::*;
// use cortex_m::peripheral::ITM;

// // A type definition for the GPIO pin to be used for our LED
// type LedPin = gpioc::PC13<Output<PushPull>>;

// // Make LED pin globally available
// static mut G_LED: Option<LedPin> = None;
// static mut G_ITM: Option<ITM> = None;
// static mut MUTEX: u32 = 1;

// #[entry]
// fn main() -> ! {

//     let mut sem: u32 = 1;

//     let dp = Peripherals::take().unwrap();

//     let rcc = dp.RCC.constrain();
//     let mut flash = dp.FLASH.constrain();
//     let clocks = rcc
//         .cfgr
//         .sysclk(8.MHz())
//         .hclk(8.MHz())
//         .pclk1(8.MHz())
//         .freeze(&mut flash.acr);

//     // Configure PC13 pin to blink LED
//     let mut gpioc = dp.GPIOC.split();
//     let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
//     let _ = led.set_low(); // Turn off


//     let cp = cortex_m::Peripherals::take().unwrap();
//     let mut itm = cp.ITM;

//     unsafe{ 
//         G_LED = Some(led);
//         G_ITM = Some(itm);
//     }


//     //HELLO OS EXAMPLE
//     let mut stack1: [u32; 500] = [1; 500];
//     let mut thread1 = OSThread::new(&mut stack1, thread_run);
//     thread1.schedule();
    
//     let mut stack2: [u32; 500] = [2; 500];
//     let mut thread2 = OSThread::new(&mut stack2, thread_run);
//     thread2.schedule();
    
//     let mut stack3: [u32; 500] = [2; 500];
//     let mut thread3 = OSThread::new(&mut stack3, thread_run);
//     thread3.schedule();
    
//     //SEMAPHORES/MUTEXES EXAMPLE
//     // let mut stack1: [u32; 500] = [1; 500];
//     // let mut thread1 = OSThread::new(&mut stack1, thread1_run);
//     // thread1.schedule();
    
//     // let mut stack2: [u32; 500] = [2; 500];
//     // let mut thread2 = OSThread::new(&mut stack2, thread2_run);
//     // thread2.schedule();
    
//     // let mut stack3: [u32; 500] = [2; 500];
//     // let mut thread3 = OSThread::new(&mut stack3, thread3_run);
//     // thread3.schedule();
    
//     //PRODUCER CONSUMER EXAMPLE
//     // let mut stack2: [u32; 500] = [2; 500];
//     // let mut thread2 = OSThread::new(&mut stack2, producer_thread);
//     // thread2.schedule();
    
//     // let mut stack3: [u32; 500] = [2; 500];
//     // let mut thread3 = OSThread::new(&mut stack3, consumer_thread);
//     // thread3.schedule();
    
//     let status : bool = os_run(8.MHz());

//     if status  {
//         // if the os succeeds to run then this is unreachable code, control will 
//         // never go back to main
//     } else {
//         // OS was not able to run since there is no
//     }

//     // if the os succeeds to run then this is unreachable code, control will never go
//     // back to main

//     loop {
//         // wfi(); // this causes issues with serial debug, do not enable for now !
//         // iprintln!(&mut itm.stim[0], "Hello, world!");
//     }
// }

// use cortex_m::itm::write_fmt;

// static mut G_COUNTER: u32 = 0;
// static mut mutex: Semaphore = Semaphore::new_mutex();

// fn thread_run() {
//     unsafe {
//         loop{
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("helloz from thread: {}\n",os_get_thread_nbr()));
//             for _ in 0..9999 {} //some delay  
//         }
//     }
// }


// fn thread1_run() {
//     unsafe {
//         let mut local;
//         loop{
//             mutex.wait();
//             local = G_COUNTER + 1;
//             for _ in 0..50 {}
//             G_COUNTER = local;
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("1:{}\n",G_COUNTER));
//             for _ in 0..50 {}
//             mutex.signal();
//             for _ in 0..99 {} //avoid starvation
            
//         }
        
//     }
// }

// fn thread2_run() {
//     unsafe {
//         let mut local;
//         loop{
//             mutex.wait();
//             local = G_COUNTER + 1;
//             for _ in 0..50 {}
//             G_COUNTER = local;
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("2:{}\n",G_COUNTER));
//             for _ in 0..50 {}
//             mutex.signal();
//             for _ in 0..99 {} //avoid starvation
            
//         }
//     }
// }

// fn thread3_run() {
//     unsafe {
//         let mut local;
//         loop{
//             mutex.wait();
//             local = G_COUNTER + 1;
//             for _ in 0..50 {}
//             G_COUNTER = local;
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("3:{}\n",G_COUNTER));
//             for _ in 0..50 {}
//             mutex.signal();
//             for _ in 0..99 {} //avoid starvation
//         }
//     }
// }


// static mut G_QUEUE : Queue<u32, 10> = Queue::new(0);
// fn producer_thread() {
//     unsafe {
//         let mut local = 0;
//         loop{
//             G_QUEUE.produce(local);
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("P:{}\n",local));
//             local = local+1;
//             // for _ in 0..999 {} // some delay for visualization
//         }
//     }
// }

// fn consumer_thread() {
//     unsafe {
//         let mut local;
//         loop{
//             local = G_QUEUE.consume();
//             write_fmt(&mut G_ITM.as_mut().unwrap().stim[0], format_args!("C:{}\n",local));
//             // for _ in 0..999 {} // some delay for visualization
//         }
//     }
// }




// fn blink() -> ! {
//     let mut led;
//     unsafe {
//         led = G_LED.as_mut().unwrap();
//     }

//     loop {
//         led.toggle();
//         for i in 0..0x00007FFF {} //fake delay
//     }
// }
