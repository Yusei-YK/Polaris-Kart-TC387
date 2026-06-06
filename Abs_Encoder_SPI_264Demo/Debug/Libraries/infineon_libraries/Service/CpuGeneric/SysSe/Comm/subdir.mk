################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.c" \
"../Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.c" 

COMPILED_SRCS += \
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.src" \
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.src" 

C_DEPS += \
"./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.d" \
"./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.d" 

OBJS += \
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.o" \
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.src":"../Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.c" "Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.o":"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.src" "Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.src":"../Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.c" "Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.o":"Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.src" "Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-Comm

clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-Comm:
	-$(RM) ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.d ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.o ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Console.src ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.d ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.o ./Libraries/infineon_libraries/Service/CpuGeneric/SysSe/Comm/Ifx_Shell.src

.PHONY: clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-SysSe-2f-Comm

