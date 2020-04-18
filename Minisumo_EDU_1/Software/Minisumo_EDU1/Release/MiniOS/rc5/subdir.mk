################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MiniOS/rc5/rc5.c 

OBJS += \
./MiniOS/rc5/rc5.o 

C_DEPS += \
./MiniOS/rc5/rc5.d 


# Each subdirectory must supply rules for building sources it contributes
MiniOS/rc5/%.o: ../MiniOS/rc5/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


