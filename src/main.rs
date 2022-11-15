/* TODO:
 * improvments can be to
 * [x] basic thread API
 * [x] the scheduler (85% of this project)!
 * [x] use systic instead of timer 1
 * [ ] setup EXCEPTION priorities
 * [x] test tail chaining, otherwise move the scheduler to systic handler
 * [x] make a thread pool and use that for the schedueler
 * [x] return an option from OSTread::new() instead of Self to check for min/max stack size
 * [ ] use slices and life times to own the stack array
 * [ ] mutex
 * [ ] semaphore
 * [ ] queues
 * [ ] OSdelay that sleeps ?
 * [ ] turn into a rust library and improve the api
 * [ ] Add documentation in github how to use the framwork
 *
 *
 *
 * later additions
 * [ ] minimal runtime wehere
 *      * all interrupts are given lower priotiy than the defult which is MAX
 *      * enable the use of thread SP and main SP to run in protected mode
 */

#![no_main]
#![no_std]

use cortex_m::iprintln;
use panic_halt as _;
use stm32f1xx_hal::{
    gpio::{gpioc, Output, PushPull},
    pac::{Peripherals},
    prelude::*,
};
// use core::cell::RefCell;
// use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;

//====================================== OS =========================================
mod OS {

    //includes
    use core::{arch::asm};
    use fugit::HertzU32 as Hertz;
    use volatile_register::RW;

    //+++++++++++++++++++++++++++ Core Peripherals ++++++++++++++++++++++++++++++++++

    #[repr(C)]
    struct SysTick {
        pub csr: RW<u32>,
        pub rvr: RW<u32>,
        pub cvr: RW<u32>,
    }

    //++++++++++++++++++++++++++++ util functions +++++++++++++++++++++++++++++++++++

    /*
    this function generates a software exception,
    WARNING: it must be protected in a critical section
    otherwize  it can be interrupted
    */
    fn generate_soft_irq() {
        // write 1 to PENDSVSET field of SCB::ICSR
        // Interrupt control and status register
        // to trigger a software interrupt
        unsafe {
            let icsr = 0xE000ED04 as *mut u32;
            let new_val = core::ptr::read_volatile(icsr) | 1 << 28;
            core::ptr::write_volatile(icsr, new_val);
        }
    }

    #[inline]
    unsafe fn __interrupts_enable() {
        asm!("CPSIE i");
    }

    #[inline]
    unsafe fn __interrupts_disable() {
        asm!("CPSID i");
    }

    //++++++++++++++++++++++++++++++ OS Structs +++++++++++++++++++++++++++++++++++++

    #[repr(C)]
    pub struct OSThread {
        //TCB
        sp: *mut u32, /* stack pointer*/
                      /* other future attributes related to the thread go here*/
                      //enabled: bool; //tells if the thread is enabled or stopped
                      //timout: u32,   //used for delay
                      //sp_init: u32,  //used for getting stack usage percentage
                      //thread_nbr:u32 //can be used by OS::getThreadNumber()
    }

    impl OSThread {
        pub fn new(stack_arr: &mut [u32], thread_runnable: fn()) -> Self {
            unsafe {
                // get the bottom of the stack
                let stack_begin = stack_arr.as_mut_ptr() as u32;

                let mut sp: u32 = stack_begin + (stack_arr.len() * 4) as u32; /* times 4  because a word is 4x1byte */

                // insure stack pointer alignment
                sp = sp / 8;
                sp = sp * 8;
                //convert to pointer
                let mut sp = sp as *mut u32;
                //fabricate stack
                (*sp.sub(1)) = 1 << 24; /* xPSR */
                (*sp.sub(2)) = thread_runnable as u32; /* PC */
                (*sp.sub(3)) = 0xBEEFCAFE; /* LR */
                (*sp.sub(4)) = 0xCAFE0012; /* R12 */
                (*sp.sub(5)) = 0xCAFE0003; /* R3 */
                (*sp.sub(6)) = 0xCAFE0002; /* R2 */
                (*sp.sub(7)) = 0xCAFE0001; /* R1 */
                (*sp.sub(8)) = 0xCAFE0000; /* R0 */
                (*sp.sub(9)) = 0xCAFE0011; /* R11 */
                (*sp.sub(10)) = 0xCAFE0010; /* R10 */
                (*sp.sub(11)) = 0xCAFE0009; /* R9 */
                (*sp.sub(12)) = 0xCAFE0008; /* R8 */
                (*sp.sub(13)) = 0xCAFE0007; /* R7 */
                (*sp.sub(14)) = 0xCAFE0006; /* R6 */
                (*sp.sub(15)) = 0xCAFE0005; /* R5 */
                (*sp.sub(16)) = 0xCAFE0004; /* R4 */

                // fill the rest of the stack with 0xF005BA11 for debugging purposes
                let mut i = 17;
                let stack_begin = stack_begin as *mut u32;
                loop {
                    if sp.sub(i) <= stack_begin {
                        (*sp.sub(i)) = 0xDEADC0DE;
                        break;
                    };
                    (*sp.sub(i)) = 0xF005BA11;
                    i += 1;
                }

                // store fake sp in TCB + make sure to store the offseted version
                sp = sp.sub(16);
                OSThread { sp: sp }
            }
        }

        pub fn schedule(&mut self) -> bool {
            unsafe{
                if thread_count < MAX_THREADS {
                    os_thread[thread_count] = self as * mut OSThread;
                    thread_count = thread_count +1;
                    true
                }else {
                    false
                }
            }
        }
    }
    //++++++++++++++++++++++++++++++ OS Variables +++++++++++++++++++++++++++++++++++
    pub static mut thread_curr: *mut OSThread = 0 as *mut OSThread; //TODO: use voletile to access these, remember you are in interrupts world
    pub static mut thread_next: *mut OSThread = 0 as *mut OSThread; //TODO: use voletile to access these, remember you are in interrupts world
    
    const MAX_THREADS : usize = 16;
    static mut thread_count : usize = 0;
    static mut thread_idx : usize = 0;
    static mut os_thread : [*mut OSThread; MAX_THREADS + 1 ] = 
                                [0 as * mut OSThread; MAX_THREADS +1]; // thread pool 
    //++++++++++++++++++++++++++++++ OS Functions +++++++++++++++++++++++++++++++++++

    pub fn os_run(sysclk: Hertz) -> bool {
        unsafe {
            if (os_thread[0] as u32) == 0 { //check if no thread was scheduled
                return false
            }
            __interrupts_disable();
            //try to calclulate SysTick Reload value register to get 4ms period
            let value = 4 * sysclk.to_kHz(); /*4 ms* f_hz = 4*f_khz */
            // configure systic
            let systick = &mut *(0xE000_E010 as *mut SysTick);
            systick.rvr.write(value);
            systick.cvr.write(0);
            systick.csr.write(0x0000_0003);
            // TODO: set priorities for pendSV and systick

            //start the scheduler (otherwize you need to wait 4ms for the timer)
            os_sched();
            __interrupts_enable();
            return true /* unreachable code here */
        }
    }

    /* Warning: this fuction must b called in a critical section */
    //Round robin scheduler lives here
    unsafe fn os_sched() {
        if thread_count == 0 {return}
        thread_next = os_thread[thread_idx];
        thread_idx = (thread_idx+1) % thread_count;
        generate_soft_irq();
    }

    #[allow(non_snake_case)]
    #[no_mangle]
    extern "C" fn PendSV() -> ! {
        unsafe {
            /* reverse llvm magic */
            asm!(
                "CPSID i",    // disable interrupts
                "pop {{r7}}", // undo llvm insertion of push {r7,lr} AND mov r7,sp
                "pop {{r7}}", // undo llvm insertion of push {r7,lr} AND r7,sp
            );

            /*save current context*/
            if thread_curr as u32 != 0 {
                asm!(
                    "push {{r4-r11}}",  /*Push regs R4-R11 to stack */
                    "STR sp, [{}]", /* get the value of sp and store into thread_curr.sp */
                    in(reg) (thread_curr as u32)
                );
            }

            /* schedule next thread */
            asm!(
                "LDR sp, [{}]",  /*load next sp from next TCB*/
                in(reg) thread_next
            );
            thread_curr = thread_next;

            /*restore next context and return*/
            asm!(
                "pop {{r4-r11}}", /*Pop regs R4-R11 to real registers */
                "CPSIE i",        //enable interrupts
                "bx lr",
                options(noreturn)
            );
        }
    }

    #[allow(non_snake_case)]
    #[no_mangle]
    extern "C" fn SysTick() -> ! {
        unsafe {
            /* reverse llvm magic */
            asm!(
                "pop {{r7}}", // undo llvm insertion of push {r7,lr} AND mov r7,sp
                "pop {{r7}}", // undo llvm insertion of push {r7,lr} AND r7,sp
            );

            /* DO STUFF */
            __interrupts_disable();
            os_sched();
            __interrupts_enable();

            //replace llvm return with expected ARM ISR return instruction
            asm!(/*"bx lr",*/
                "bx {}",
                in(reg) 0xfffffff9 as u32,
                options(noreturn));
        }
    }


}

//===================================================================================

use OS::*;

// A type definition for the GPIO pin to be used for our LED
type LedPin = gpioc::PC13<Output<PushPull>>;

// Make LED pin globally available
static mut G_LED: Option<LedPin> = None;


#[entry]
fn main() -> ! {
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
    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let _ = led.set_low(); // Turn off


    let cp = cortex_m::Peripherals::take().unwrap();
    let mut itm = cp.ITM;

    let mut stack1: [u32; 100] = [1; 100];
    let mut thread1 = OSThread::new(&mut stack1, thread1_run);
    thread1.schedule();
    
    let mut stack2: [u32; 100] = [2; 100];
    let mut thread2 = OSThread::new(&mut stack2, thread2_run);
    thread2.schedule();
    
    let mut stack3: [u32; 100] = [2; 100];
    let mut thread3 = OSThread::new(&mut stack3, thread3_run);
    thread3.schedule();
    
    let status : bool = OS::os_run(8.MHz());

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

static mut G_COUNT: u32 = 0;

fn thread1_run() {
    let mut id = 1;
    unsafe {
        loop {
            G_COUNT = id;
            id += 2;
        }
    }
}

fn thread2_run() {
    let mut id = 2;
    unsafe {
        loop {
            G_COUNT = id;
            id += 2;
        }
    }
}


fn thread3_run() {
    let mut id = 3;
    unsafe {
        loop {
            G_COUNT = id;
            id += 3;
        }
    }
}


fn blink() -> ! {
    let mut led;
    unsafe {
        led = G_LED.as_mut().unwrap();
    }

    loop {
        led.toggle();
        for i in 0..0x00007FFF {} //fake delay
    }
}
