################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.c" \
"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.c" \
"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.c" \
"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.c" 

COMPILED_SRCS += \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.src" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.src" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.src" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.src" 

C_DEPS += \
"./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.d" \
"./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.d" \
"./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.d" \
"./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.d" 

OBJS += \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.o" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.o" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.o" \
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.src":"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.c" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.o":"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.src" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.src":"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.c" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.o":"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.src" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.src":"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.c" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.o":"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.src" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.src":"../Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.c" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.o":"Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.src" "Libraries/infineon_libraries/Service/CpuGeneric/StdIf/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-StdIf

clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-StdIf:
	-$(RM) ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.d ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.o ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_DPipe.src ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.d ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.o ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Pos.src ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.d ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.o ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_PwmHl.src ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.d ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.o ./Libraries/infineon_libraries/Service/CpuGeneric/StdIf/IfxStdIf_Timer.src

.PHONY: clean-Libraries-2f-infineon_libraries-2f-Service-2f-CpuGeneric-2f-StdIf

