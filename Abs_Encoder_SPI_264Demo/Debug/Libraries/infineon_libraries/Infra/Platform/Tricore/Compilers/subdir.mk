################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.c" \
"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.c" \
"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.c" \
"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.c" 

COMPILED_SRCS += \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.src" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.src" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.src" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.src" 

C_DEPS += \
"./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.d" \
"./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.d" \
"./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.d" \
"./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.d" 

OBJS += \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.o" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.o" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.o" \
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.o" 


# Each subdirectory must supply rules for building sources it contributes
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.src":"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.c" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.o":"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.src" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.src":"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.c" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.o":"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.src" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.src":"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.c" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.o":"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.src" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.src":"../Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.c" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc26xb "-fD:/Github/SmartCar/Abs_Encoder_SPI_264Demo/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<"
"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.o":"Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.src" "Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-Libraries-2f-infineon_libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers

clean-Libraries-2f-infineon_libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers:
	-$(RM) ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.d ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.o ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerDcc.src ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.d ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.o ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGhs.src ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.d ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.o ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerGnuc.src ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.d ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.o ./Libraries/infineon_libraries/Infra/Platform/Tricore/Compilers/CompilerTasking.src

.PHONY: clean-Libraries-2f-infineon_libraries-2f-Infra-2f-Platform-2f-Tricore-2f-Compilers

