################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../USER/Cpu0_Main.c" \
"../USER/Cpu1_Main.c" \
"../USER/isr.c" 

COMPILED_SRCS += \
"USER/Cpu0_Main.src" \
"USER/Cpu1_Main.src" \
"USER/isr.src" 

C_DEPS += \
"./USER/Cpu0_Main.d" \
"./USER/Cpu1_Main.d" \
"./USER/isr.d" 

OBJS += \
"USER/Cpu0_Main.o" \
"USER/Cpu1_Main.o" \
"USER/isr.o" 


# Each subdirectory must supply rules for building sources it contributes
"USER/Cpu0_Main.src":"../USER/Cpu0_Main.c" "USER/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"USER/Cpu0_Main.o":"USER/Cpu0_Main.src" "USER/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"USER/Cpu1_Main.src":"../USER/Cpu1_Main.c" "USER/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"USER/Cpu1_Main.o":"USER/Cpu1_Main.src" "USER/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"USER/isr.src":"../USER/isr.c" "USER/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"USER/isr.o":"USER/isr.src" "USER/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-USER

clean-USER:
	-$(RM) ./USER/Cpu0_Main.d ./USER/Cpu0_Main.o ./USER/Cpu0_Main.src ./USER/Cpu1_Main.d ./USER/Cpu1_Main.o ./USER/Cpu1_Main.src ./USER/isr.d ./USER/isr.o ./USER/isr.src

.PHONY: clean-USER

