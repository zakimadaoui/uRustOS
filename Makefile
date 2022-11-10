 
BUILD_DIR=./target/thumbv7m-none-eabi
TARGET=uRustOS


asm:
	arm-none-eabi-objdump -D $(BUILD_DIR)/release/$(TARGET) > release.asm
	arm-none-eabi-objdump -D $(BUILD_DIR)/debug/$(TARGET) > debug.asm


