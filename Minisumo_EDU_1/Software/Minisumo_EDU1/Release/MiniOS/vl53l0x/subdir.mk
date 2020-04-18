################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MiniOS/vl53l0x/VL53L0X.c 

OBJS += \
./MiniOS/vl53l0x/VL53L0X.o 

C_DEPS += \
./MiniOS/vl53l0x/VL53L0X.d 


# Each subdirectory must supply rules for building sources it contributes
MiniOS/vl53l0x/%.o: ../MiniOS/vl53l0x/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


