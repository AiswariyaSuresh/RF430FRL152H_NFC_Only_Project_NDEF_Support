################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./NDEF.obj" \
"./main.obj" \
"./patch.obj" \
"../lnk_rf430frl152h_NFC_Only.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
RF430FRL152H_NFC_Only_Project_NDEF_Support.out \

EXE_OUTPUTS__QUOTED += \
"RF430FRL152H_NFC_Only_Project_NDEF_Support.out" \

BIN_OUTPUTS += \
RF430FRL152H_NFC_Only_Project_NDEF_Support.txt \

BIN_OUTPUTS__QUOTED += \
"RF430FRL152H_NFC_Only_Project_NDEF_Support.txt" \


# All Target
all: RF430FRL152H_NFC_Only_Project_NDEF_Support.out

# Tool invocations
RF430FRL152H_NFC_Only_Project_NDEF_Support.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: MSP430 Linker'
	"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS/bin/cl430" -vmsp --abi=eabi -O3 --advice:power="all" -g --define=__RF430FRL152H__ --diag_warning=225 --display_error_number --diag_suppress=10325 --diag_suppress=2591 --diag_wrap=off --silicon_errata=CPU21 --silicon_errata=CPU22 --silicon_errata=CPU40 --printf_support=minimal --asm_listing -z -m"RF430FRL152H_NFC_Only_Project_NDEF_Support.map" --heap_size=0 --stack_size=100 --cinit_hold_wdt=off -i"C:/ti/ccs1031/ccs/ccs_base/msp430/include" -i"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS/lib" -i"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="RF430FRL152H_NFC_Only_Project_NDEF_Support_linkInfo.xml" --rom_model -o "RF430FRL152H_NFC_Only_Project_NDEF_Support.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

RF430FRL152H_NFC_Only_Project_NDEF_Support.txt: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: MSP430 Hex Utility'
	"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS/bin/hex430" --memwidth=8 --romwidth=8 --diag_wrap=off -o "RF430FRL152H_NFC_Only_Project_NDEF_Support.txt" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "NDEF.lst" "main.lst" "patch.lst" 
	-$(RM) "NDEF.obj" "main.obj" "patch.obj" 
	-$(RM) "NDEF.d" "main.d" "patch.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-@echo 'Create flash image: TI-TXT'
	-"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-msp430_20.2.4.LTS/bin/hex430" --ti_txt "RF430FRL152H_NFC_Only_Project_NDEF_Support.out" -o "RF430FRL152H_NFC_Only_Project_NDEF_Support.txt" -order MS -romwidth 16
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

