################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.c" 

COMPILED_SRCS += \
"Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.src" 

C_DEPS += \
"./Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.d" 

OBJS += \
"Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.src":"../Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.c" "Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.o":"Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.src" "Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Port-2f-Std

clean-Libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Port-2f-Std:
	-$(RM) ./Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.d ./Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.o ./Libraries/infineon_libraries/iLLD/TC26B/Tricore/Port/Std/IfxPort.src

.PHONY: clean-Libraries-2f-infineon_libraries-2f-iLLD-2f-TC26B-2f-Tricore-2f-Port-2f-Std

