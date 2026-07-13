#ifndef TEST_ZF_COMMON_HEADFILE_H_
#define TEST_ZF_COMMON_HEADFILE_H_

#include <stddef.h>
#include <stdint.h>

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t   int8;
typedef int16_t  int16;
typedef int32_t  int32;

#define PWM_DUTY_MAX (10000)

uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 primask);

#endif
