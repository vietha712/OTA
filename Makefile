PRO_DIR := .
PROJ_NAME := stm32f411_gpio_output
OUTPUT_PATH := $(PRO_DIR)/output

INC_DIR := $(PRO_DIR)/Inc
SRC_DIR := $(PRO_DIR)/Src
LINKER_FILE := $(PRO_DIR)/linker/gcc_arm.ld

#GPIO_driver file
GPIO_COMPILED_FILE := E:/Workspace_window/GPIO_driver/Compiled/gpio_driver.o
GPIO_INC_DIR := E:/Workspace_window/GPIO_driver/Inc

#Your downloaded compiler folder
COMPILER_DIR := F:/GNU_Tools/72018-q2-update

PREFIX_GCC_COMPILER := arm-none-eabi
CC := $(COMPILER_DIR)/bin/$(PREFIX_GCC_COMPILER)-gcc
OBJCPY := $(COMPILER_DIR)/bin/$(PREFIX_GCC_COMPILER)-objcopy
ASM := $(COMPILER_DIR)/bin/$(PREFIX_GCC_COMPILER)-as
LD := $(COMPILER_DIR)/bin/$(PREFIX_GCC_COMPILER)-ld

FILE_TO_LINK := $(OUTPUT_PATH)/main.o $(OUTPUT_PATH)/function.o $(OUTPUT_PATH)/startup_ARMCM4.o $(GPIO_COMPILED_FILE)

#Compiler options
CC_OPT := -mthumb -mcpu=cortex-m4 -c -O0 -g -Wall -I$(INC_DIR) -I$(GPIO_INC_DIR)
#Assembler options
ASM_OPT := -mthumb -mcpu=cortex-m4 -c
#Linker options
LD_OPT := -T $(LINKER_FILE) -Map $(OUTPUT_PATH)/$(PROJ_NAME).map $(FILE_TO_LINK)

#Build process to build a c program
$(OUTPUT_PATH)/main.o: $(SRC_DIR)/main.c
	@echo "compile $(SRC_DIR)/main.c file"
	$(CC) $(CC_OPT) $(SRC_DIR)/main.c -o $(OUTPUT_PATH)/main.o

$(OUTPUT_PATH)/function.o: $(SRC_DIR)/function.c
	@echo "compile $(SRC_DIR)/function.c file"
	$(CC) $(CC_OPT) $(SRC_DIR)/function.c -o $(OUTPUT_PATH)/function.o

#Compile startup code
$(OUTPUT_PATH)/startup_ARMCM4.o: $(SRC_DIR)/startup_ARMCM4.S
	@echo "compile $(SRC_DIR)/startup_ARMCM4.s file"
	$(ASM) $(ASM_OPT) $(SRC_DIR)/startup_ARMCM4.S -o $(OUTPUT_PATH)/startup_ARMCM4.o

#Makefile command
build: $(FILE_TO_LINK) $(LINKER_FILE)
	@echo "--> Linking all object file and creating .elf file"
	$(LD) $(LD_OPT) -o $(OUTPUT_PATH)/$(PROJ_NAME).elf

delete_o:
	@rm -rf $(OUTPUT_PATH)/*
