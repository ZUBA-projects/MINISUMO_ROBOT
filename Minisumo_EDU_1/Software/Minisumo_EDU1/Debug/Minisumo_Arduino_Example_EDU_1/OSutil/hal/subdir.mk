################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Minisumo_Arduino_Example_EDU_1/OSutil/hal/adc.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/hal/eeprom.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/hal/gpio.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/hal/i2c.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/hal/pwm.c 

OBJS += \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/adc.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/eeprom.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/gpio.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/i2c.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/pwm.o 

C_DEPS += \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/adc.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/eeprom.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/gpio.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/i2c.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/hal/pwm.d 


# Each subdirectory must supply rules for building sources it contributes
Minisumo_Arduino_Example_EDU_1/OSutil/hal/%.o: ../Minisumo_Arduino_Example_EDU_1/OSutil/hal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


