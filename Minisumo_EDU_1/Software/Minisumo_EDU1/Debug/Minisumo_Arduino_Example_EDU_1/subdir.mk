################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Minisumo_Arduino_Example_EDU_1/miniOS.c 

OBJS += \
./Minisumo_Arduino_Example_EDU_1/miniOS.o 

C_DEPS += \
./Minisumo_Arduino_Example_EDU_1/miniOS.d 


# Each subdirectory must supply rules for building sources it contributes
Minisumo_Arduino_Example_EDU_1/%.o: ../Minisumo_Arduino_Example_EDU_1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


