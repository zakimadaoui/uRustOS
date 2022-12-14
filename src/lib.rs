#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]

/* Compilation commands:
 * cargo embed --example hello_os
 * 
 * to only build all examples:
 * cargo build --examples
 * 
 * more specific onesL 
 * cargo build --example hello_os 
 * cargo flash --chip stm32f103C8 --example hello_os
 * 
*/

/* TODOs:
 * [x] basic thread API
 * [x] the scheduler (85% of this project)!
 * [x] use systic instead of timer 1
 * [x] test tail chaining, otherwise move the scheduler to systic handler
 * [x] make a thread pool and use that for the schedueler
 * [x] return an option from OSTread::new() instead of Self to check for min/max stack size
 * [x] mutex
 * [x] semaphore
 * [x] non blocking OSdelay (run other threads while waiting) ?
 * [x] queues
 * [x] thred nbr, stop and start. + add it to example
 * [x] turn into a rust library and run examples independently
 * [x] FIX SLEEP & MAKE BLINKY_OS EXAMPLE
 * [x] re-enable _wfi and test if all is alright !?
 * [ ] setup EXCEPTION priorities for tailchaining pendSV and Systick
 * [-] Add good documentation in github how to use the framework
 *
 *
 *
 * future improvments
 * [ ] own the stack array to avoid copy-past stack bugs
 * [ ] add priority scheduling
 * [ ] minimal runtime wehere:
 *      * all interrupts are given lower priority than the defult which is MAX
 *      * enable the use of thread SP and main SP to run in protected mode
 */

#![no_std]


//====================================== OS =========================================
pub mod OS {
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
    pub unsafe fn __interrupts_enable() {
        asm!("CPSIE i");
    }

    #[inline]
    pub unsafe fn __interrupts_disable() {
        asm!("CPSID i");
    }

    pub fn os_get_thread_nbr() -> usize {
        unsafe{thread_idx}
    }

    pub fn os_get_threads_count() -> usize {
        unsafe{thread_count}
    }

    //++++++++++++++++++++++++++++++ OS Structs +++++++++++++++++++++++++++++++++++++

    #[repr(C)]
    pub struct OSThread {
        //TCB
        sp: *mut u32, /* stack pointer*/
        timeout : u32, /* used for delay funcionality */
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
                OSThread { sp: sp, timeout : 0 }
            }
        }

        pub fn schedule(&mut self) -> bool {
            unsafe{
                if thread_count < MAX_THREADS {
                    os_thread[thread_count] = self as * mut OSThread;
                    is_ready_flags |= 0x1 << (thread_count -1);
                    thread_count = thread_count +1;
                    true
                }else {
                    false
                }
            }
        }
    }


    pub struct Semaphore{
        lock_counter: u32,
    }
    impl Semaphore {
        pub const fn new_sem(init_val: u32) -> Self{
            Semaphore{lock_counter:init_val}
        }

        pub const fn new_mutex() -> Self{
            Semaphore{lock_counter:1}
        }

        pub fn wait(&mut self){
            unsafe{
                // let mut val : u32 = 77;
                asm!(
                    "1:",
                    "MOV     R5,#0x1",
                    "LDREX   R4, [{0}]",     // load sem value and tag memory location
                    "CMP     R4, #0",        // is semaphore consumed ?
                    "ITTT    NE",            // IT block for next 3 instructions
                    "SUBNE   R4, R4, #1",    // dec sem IF sem is not 0
                    "STREXNE R5, R4, [{0}]", // store exclusive IF sem is not 0
                    "CMPNE   R5, #0",        // see if operation is successful
                    "BNE     2f",            // break from the loop if successful
                    "CMP     R5, #0",
                    "BEQ     2f",            
                    in(reg) &mut self.lock_counter as *mut u32
                );
                // TODO: you can disable this section to create a spin lock !
                // __interrupts_disable();
                //os_sched();
                // __interrupts_enable();
                asm!(
                    "B      1b",//retry by looping
                    "2:     ", //break loop lable
                );
            }
        }

        pub fn wait2(&mut self){
            unsafe{
                loop{
                    __interrupts_disable();
                    if self.lock_counter != 0 {
                        self.lock_counter = self.lock_counter -1;
                        break;
                    }
                    os_sched();
                    __interrupts_enable();
                }

            }
        }
        
        pub fn signal(&mut self){
            unsafe{
                // let mut val : u32 = 77;
                asm!(
                    "1:",
                    "MOV     R5,#0x1",
                    "LDREX   R4, [{0}]",     // load sem value and tag memory location
                    "ADD     R4, R4, #1",    // dec sem IF sem is not 0
                    "STREX   R5, R4, [{0}]", // store exclusive IF sem is not 0
                    "CMP     R5, #0",        // see if operation is successful
                    "BEQ     2f",            // break from the loop if successful        
                    in(reg) &mut self.lock_counter as *mut u32
                );
                asm!(
                    "B      1b",//retry by looping (this should never occur, even tests prove it)
                    "2:     ", //break loop lable
                );
            }
        }
        pub fn signal2(&mut self){
            unsafe{
                __interrupts_disable();
                self.lock_counter = self.lock_counter +1;
                __interrupts_enable();

            }
        }
    }


    pub struct Queue<T:Copy, const N:usize>{
        queueArr: [T; N],
        full: Semaphore,
        empty: Semaphore,
        mutex: Semaphore,
        count: usize,
        max_count: usize,
        head_idx: usize,
        tail_idx: usize,
    }

    impl<T:Copy, const N:usize> Queue<T, N> {
        pub const fn new(def_val: T) -> Self{
            Self{
                queueArr: [def_val; N],
                full: Semaphore::new_sem(0),
                empty: Semaphore::new_sem(N as u32),
                mutex: Semaphore::new_mutex(),
                count: 0,
                max_count: N,
                head_idx:  0,
                tail_idx:  0,
            }
        }
        
        pub fn produce(&mut self, val: T){

            self.empty.wait(); //wait if not empty (ie. full)
            self.mutex.wait(); //cs begin
            
            //produce start
            self.queueArr[self.head_idx] = val;
            self.head_idx = (self.head_idx +1)%self.max_count;
            self.count = self.count+ 1;
            //produce end
            
            self.mutex.signal();
            self.full.signal();
        }

        pub fn consume(&mut self) -> T {
            let val : T;
            self.full.wait();  //wait if empty
            self.mutex.wait(); //cs begin
            
            //consume here
            val = self.queueArr[self.tail_idx];
            self.tail_idx = (self.tail_idx +1)%self.max_count;
            self.count = self.count - 1 ;

            self.mutex.signal(); //cs end
            self.empty.signal(); // signal that one element has been freed
            val
        }

        pub fn size(&mut self) -> usize{
            self.count
        }
    }
    

    //++++++++++++++++++++++++++++++ OS Variables +++++++++++++++++++++++++++++++++++
    pub static mut thread_curr: *mut OSThread = 0 as *mut OSThread; 
    pub static mut thread_next: *mut OSThread = 0 as *mut OSThread; 

    static mut is_ready_flags : u16 = 0;

    const MAX_THREADS : usize = 16;
    const TIME_SLICE : u32 = 4;
    static mut thread_count : usize = 1;
    static mut thread_idx : usize = 0; //current thread index
    static mut os_thread : [*mut OSThread; MAX_THREADS + 1 ] = 
                                [0 as * mut OSThread; MAX_THREADS +1]; // thread pool
                                
    static mut idle_thread_stack : [u32; 40] = [0; 40];
    static mut idle_thread: OSThread = OSThread {sp:0 as *mut u32, timeout:0};
    //++++++++++++++++++++++++++++++ OS Functions +++++++++++++++++++++++++++++++++++

    pub fn os_run(sysclk: Hertz) -> bool {
        unsafe {
            if (os_thread[1] as u32) == 0 { //check if no thread was scheduled
                return false
            }
            __interrupts_disable();
            //try to calclulate SysTick Reload value register to get 4ms period
            let value = TIME_SLICE * sysclk.to_kHz(); /*4 ms* f_hz = 4*f_khz */
            // configure systic
            let systick = &mut *(0xE000_E010 as *mut SysTick);
            systick.rvr.write(value);
            systick.cvr.write(0);
            systick.csr.write(0x0000_0007);
            // TODO: set priorities for pendSV and systick
            
            //schedule idle thread
            idle_thread = OSThread::new(&mut idle_thread_stack, idle_thread_handler);
            os_thread[0] = &mut idle_thread as * mut OSThread;

            //start the scheduler (otherwize you need to wait 4ms for the timer)
            os_sched();
            __interrupts_enable();
            return true /* unreachable code here */
        }
    }

    fn idle_thread_handler(){
        loop{
            unsafe {asm!("wfi");}
            // unsafe {asm!("nop");}
        }
    }

    /* this function puts the caller thread to sleep untill the delay time elapses
    the delay should be above 4*num_threads milliseconds to be accurate.
    WARNING: never call this function in main (ie, before os_run()) */
    pub fn os_sleep(mut delay: u32){
        unsafe {
            __interrupts_disable();
            //correct delay to 4ms accuracy
            delay = delay / TIME_SLICE ;
            delay = delay * TIME_SLICE ;
            //check for null (maybe someone calls this in main)
            if thread_curr as u32 == 0 {return} 
            //update thread timeout feild
            (*thread_curr).timeout = delay;
            //set thread as no ready
            is_ready_flags &= !(0x1 << (thread_idx -1)); //clear is ready flag
            os_sched();//call the scheduler
            __interrupts_enable();
        }
    }

    unsafe fn os_tick(){
        let mut i  = 1;
        loop{
            if i == thread_count  {break;}
            if (*os_thread[i]).timeout > 0 {
                (*os_thread[i]).timeout-=4;
                
                if (*os_thread[i]).timeout == 0 {
                    is_ready_flags |= 0x1 << (i -1); //set thread to ready
                }   
            }
            i+=1;
        }
    }

    /* Warning: this fuction must b called in a critical section */
    //Round robin scheduler lives here
    unsafe fn os_sched() {
        if is_ready_flags == 0 {
            thread_idx = 0;
        } else { //look for the next ready thread
            loop{
                thread_idx = (thread_idx) % thread_count +1;
                if ( is_ready_flags & 0x1<<(thread_idx-1) ) != 0 {break;}
            }
        }
        thread_next = os_thread[thread_idx];
        if thread_next!= thread_curr {generate_soft_irq();} //call pendSV to run next thead
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
            os_tick();
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

