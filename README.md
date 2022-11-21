# uRusTOS: a minimal real-time operating system written (mostly) in rust
This is an experimental/educational project which tries to build an RTOS almost fully in Rust Lang for the famous stm32f103 MCU.
Currently the OS offers:

- Round-Robin task scheduler with up to 16 Threads
- Efficient thread sleep/delay syscall
- Zero interrupt latency synchronization primitives
    - Mutex
    - Semaphore
    - Queue (Producer Consumer pattern)
- utility functions for getting thread index and count.


Please check the examples to see how to get the OS running ! and how to use its API.


## Future improvments:
[ ] stack usage for threads
[ ] priority based scheduling
[ ] safe sync premitives (currently can only be called in unsafe blocks)

## Warnings:

* Due to the fact that embedded-rust offers no heap allocation, the thread's stack must be allocated in main, and then start the os which never returns to main. This is in order to keep the alocated stack alive.

* Since the thread's stack is stack-allocated (i know, a bit confusing) you should never ever create a thread from another one. All threads must be created in main() before starting the OS.  
