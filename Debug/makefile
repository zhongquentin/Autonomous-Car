################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./EEC195Bmain.obj" \
"./startup_ccs.obj" \
"./driverlib/adc.obj" \
"./driverlib/epi_workaround_ccs.obj" \
"./driverlib/timer.obj" \
"./driverlib/uart.obj" \
"./driverlib/udma.obj" \
"./driverlib/usb.obj" \
"./driverlib/watchdog.obj" \
"./utils/uartstdio.obj" \
"../timers_ccs.cmd" \
"../driverlib/ccs/Debug/driverlib.lib" \
"../driverlib/rvmdk/driverlib.lib" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/TivaWare_C_Series-2.1.3.156/driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include driverlib/subdir_vars.mk
-include driverlib/ccs/Debug/subdir_vars.mk
-include driverlib/ewarm/Exe/subdir_vars.mk
-include driverlib/gcc/subdir_vars.mk
-include driverlib/rvmdk/subdir_vars.mk
-include utils/subdir_vars.mk
-include subdir_rules.mk
-include driverlib/subdir_rules.mk
-include driverlib/ccs/Debug/subdir_rules.mk
-include driverlib/ewarm/Exe/subdir_rules.mk
-include driverlib/gcc/subdir_rules.mk
-include driverlib/rvmdk/subdir_rules.mk
-include utils/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out \

EXE_OUTPUTS__QUOTED += \
"FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out" \

BIN_OUTPUTS += \
FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.bin \

BIN_OUTPUTS__QUOTED += \
"FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.bin" \


# All Target
all: FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out

# Tool invocations
FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(A_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -O2 --advice:power=all -g --gcc --define=ccs="ccs" --define=PART_TM4C123GH6PM --define=TARGET_IS_TM4C123_RB1 --diag_wrap=off --diag_warning=225 --display_error_number --gen_func_subsections=on --abi=eabi --ual -z -m"timers_ccs.map" --stack_size=2048 --heap_size=0 -i"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/lib" -i"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/include" --reread_libs --diag_wrap=off --warn_sections --display_error_number --xml_link_info="FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec_linkInfo.xml" --rom_model -o "FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out" $(ORDERED_OBJS) $(A_SRCS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.bin: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armhex"  -o "FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.bin" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "EEC195Bmain.d" "startup_ccs.d" "driverlib\adc.d" "driverlib\timer.d" "driverlib\uart.d" "driverlib\udma.d" "driverlib\usb.d" "driverlib\watchdog.d" "utils\uartstdio.d" 
	-$(RM) "driverlib\epi_workaround_ccs.d" 
	-$(RM) "EEC195Bmain.obj" "startup_ccs.obj" "driverlib\adc.obj" "driverlib\epi_workaround_ccs.obj" "driverlib\timer.obj" "driverlib\uart.obj" "driverlib\udma.obj" "driverlib\usb.obj" "driverlib\watchdog.obj" "utils\uartstdio.obj" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"C:/ti/ccsv6/utils/tiobj2bin/tiobj2bin" "FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.out" "FINAL_CODE_TimeTrial_1_EEC195B_9.7ftpersec.bin" "C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armofd" "C:/ti/ccsv6/tools/compiler/arm_15.12.3.LTS/bin/armhex" "C:/ti/ccsv6/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

