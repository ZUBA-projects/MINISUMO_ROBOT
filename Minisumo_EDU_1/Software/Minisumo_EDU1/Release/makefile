################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include MiniOS/vl53l0x/subdir.mk
-include MiniOS/rc5/subdir.mk
-include MiniOS/oled/subdir.mk
-include MiniOS/motor/subdir.mk
-include MiniOS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Minisumo_EDU1.lss \

FLASH_IMAGE += \
Minisumo_EDU1.hex \

EEPROM_IMAGE += \
Minisumo_EDU1.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Minisumo_EDU1.elf secondary-outputs

# Tool invocations
Minisumo_EDU1.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Minisumo_EDU1.map -mmcu=atmega328p -o "Minisumo_EDU1.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Minisumo_EDU1.lss: Minisumo_EDU1.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Minisumo_EDU1.elf  >"Minisumo_EDU1.lss"
	@echo 'Finished building: $@'
	@echo ' '

Minisumo_EDU1.hex: Minisumo_EDU1.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Minisumo_EDU1.elf  "Minisumo_EDU1.hex"
	@echo 'Finished building: $@'
	@echo ' '

Minisumo_EDU1.eep: Minisumo_EDU1.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Minisumo_EDU1.elf  "Minisumo_EDU1.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Minisumo_EDU1.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Minisumo_EDU1.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Minisumo_EDU1.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
