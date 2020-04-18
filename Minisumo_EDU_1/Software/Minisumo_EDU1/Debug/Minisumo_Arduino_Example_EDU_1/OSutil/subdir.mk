################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Minisumo_Arduino_Example_EDU_1/OSutil/ktir0711s.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/memory.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/motor.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/rc5.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/ssd1306.c \
../Minisumo_Arduino_Example_EDU_1/OSutil/vl53l0x.c 

OBJS += \
./Minisumo_Arduino_Example_EDU_1/OSutil/ktir0711s.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/memory.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/motor.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/rc5.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/ssd1306.o \
./Minisumo_Arduino_Example_EDU_1/OSutil/vl53l0x.o 

C_DEPS += \
./Minisumo_Arduino_Example_EDU_1/OSutil/ktir0711s.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/memory.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/motor.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/rc5.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/ssd1306.d \
./Minisumo_Arduino_Example_EDU_1/OSutil/vl53l0x.d 


# Each subdirectory must supply rules for building sources it contributes
Minisumo_Arduino_Example_EDU_1/OSutil/%.o: ../Minisumo_Arduino_Example_EDU_1/OSutil/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=1600000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


