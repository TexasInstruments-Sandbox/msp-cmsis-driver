/*
 * Copyright (C) 2024-2025 Texas Instruments Incorporated
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and  
 * limitations under the License.
 */

/*!****************************************************************************
 *  @file       Driver_GPIO_MSP.c
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_GPIO_MSP_Priv.h>
#include <Driver_GPIO_MSP.h>

#if defined(__MSPM0G3507__)
#if defined(GPIOA_BASE) && (DRIVER_CONFIG_HAS_GPIOA==1)
uint8_t PORTA_PINCMs[] = {IOMUX_PINCM1, IOMUX_PINCM2, IOMUX_PINCM7, 
                          IOMUX_PINCM8, IOMUX_PINCM9, IOMUX_PINCM10, 
                          IOMUX_PINCM11, IOMUX_PINCM14, IOMUX_PINCM19, 
                          IOMUX_PINCM20, IOMUX_PINCM21, IOMUX_PINCM22, 
                          IOMUX_PINCM34, IOMUX_PINCM35, IOMUX_PINCM36, 
                          IOMUX_PINCM37, IOMUX_PINCM38, IOMUX_PINCM39, 
                          IOMUX_PINCM40, IOMUX_PINCM41, IOMUX_PINCM42, 
                          IOMUX_PINCM46, IOMUX_PINCM47, IOMUX_PINCM53, 
                          IOMUX_PINCM54, IOMUX_PINCM55, IOMUX_PINCM59, 
                          IOMUX_PINCM60, IOMUX_PINCM3, IOMUX_PINCM4,
                          IOMUX_PINCM5, IOMUX_PINCM6};
#endif //Has GPIOA and GPIOA enabled
#if defined(GPIOB_BASE) && (DRIVER_CONFIG_HAS_GPIOB==1)
uint8_t PORTB_PINCMs[] = {IOMUX_PINCM12, IOMUX_PINCM13, IOMUX_PINCM14, 
                          IOMUX_PINCM15, IOMUX_PINCM15, IOMUX_PINCM16, 
                          IOMUX_PINCM23, IOMUX_PINCM24, IOMUX_PINCM25, 
                          IOMUX_PINCM26, IOMUX_PINCM27, IOMUX_PINCM28, 
                          IOMUX_PINCM29, IOMUX_PINCM30, IOMUX_PINCM31, 
                          IOMUX_PINCM32, IOMUX_PINCM33, IOMUX_PINCM43, 
                          IOMUX_PINCM44, IOMUX_PINCM45, IOMUX_PINCM46, 
                          IOMUX_PINCM49, IOMUX_PINCM50, IOMUX_PINCM51, 
                          IOMUX_PINCM52, IOMUX_PINCM56, IOMUX_PINCM57, 
                          IOMUX_PINCM58};
#endif //Has GPIOB and GPIOB enabled
#elif defined(__MSPM0G3519__)
#if defined(GPIOA_BASE) && (DRIVER_CONFIG_HAS_GPIOA==1)
uint8_t PORTA_PINCMs[] = {IOMUX_PINCM1, IOMUX_PINCM2, IOMUX_PINCM7, 
                          IOMUX_PINCM8, IOMUX_PINCM9, IOMUX_PINCM10, 
                          IOMUX_PINCM11, IOMUX_PINCM14, IOMUX_PINCM19, 
                          IOMUX_PINCM20, IOMUX_PINCM21, IOMUX_PINCM22, 
                          IOMUX_PINCM34, IOMUX_PINCM35, IOMUX_PINCM36, 
                          IOMUX_PINCM37, IOMUX_PINCM38, IOMUX_PINCM39, 
                          IOMUX_PINCM40, IOMUX_PINCM41, IOMUX_PINCM42, 
                          IOMUX_PINCM46, IOMUX_PINCM47, IOMUX_PINCM53, 
                          IOMUX_PINCM54, IOMUX_PINCM55, IOMUX_PINCM59, 
                          IOMUX_PINCM60, IOMUX_PINCM3, IOMUX_PINCM4,
                          IOMUX_PINCM5, IOMUX_PINCM6};
#endif //Has GPIOA and GPIOA enabled
#if defined(GPIOB_BASE) && (DRIVER_CONFIG_HAS_GPIOB==1)
uint8_t PORTB_PINCMs[] = {IOMUX_PINCM12, IOMUX_PINCM13, IOMUX_PINCM15, 
                          IOMUX_PINCM16, IOMUX_PINCM17, IOMUX_PINCM18, 
                          IOMUX_PINCM23, IOMUX_PINCM24, IOMUX_PINCM25, 
                          IOMUX_PINCM26, IOMUX_PINCM27, IOMUX_PINCM28, 
                          IOMUX_PINCM29, IOMUX_PINCM30, IOMUX_PINCM31, 
                          IOMUX_PINCM32, IOMUX_PINCM33, IOMUX_PINCM43, 
                          IOMUX_PINCM44, IOMUX_PINCM45, IOMUX_PINCM48, 
                          IOMUX_PINCM49, IOMUX_PINCM50, IOMUX_PINCM51, 
                          IOMUX_PINCM52, IOMUX_PINCM56, IOMUX_PINCM57, 
                          IOMUX_PINCM58, IOMUX_PINCM65, IOMUX_PINCM66,
                          IOMUX_PINCM67, IOMUX_PINCM68};
#endif //Has GPIOB and GPIOB enabled
#if defined(GPIOC_BASE) && (DRIVER_CONFIG_HAS_GPIOC==1)
uint8_t PORTC_PINCMs[] = {IOMUX_PINCM74, IOMUX_PINCM75, IOMUX_PINCM76, 
                          IOMUX_PINCM77, IOMUX_PINCM78, IOMUX_PINCM79, 
                          IOMUX_PINCM84, IOMUX_PINCM85, IOMUX_PINCM86, 
                          IOMUX_PINCM87, IOMUX_PINCM88, IOMUX_PINCM89, 
                          IOMUX_PINCM61, IOMUX_PINCM62, IOMUX_PINCM63, 
                          IOMUX_PINCM64, IOMUX_PINCM69, IOMUX_PINCM70, 
                          IOMUX_PINCM71, IOMUX_PINCM72, IOMUX_PINCM73, 
                          IOMUX_PINCM80, IOMUX_PINCM81, IOMUX_PINCM82, 
                          IOMUX_PINCM83, IOMUX_PINCM90, IOMUX_PINCM91, 
                          IOMUX_PINCM92, IOMUX_PINCM93, IOMUX_PINCM94};
#endif //Has GPIOC and GPIOC enabled
#elif defined(__MSPM0L2228__)
#if defined(GPIOA_BASE) && (DRIVER_CONFIG_HAS_GPIOA==1)
uint8_t PORTA_PINCMs[] = {IOMUX_PINCM1, IOMUX_PINCM2, IOMUX_PINCM7, 
                          IOMUX_PINCM8, IOMUX_PINCM9, IOMUX_PINCM10, 
                          IOMUX_PINCM11, IOMUX_PINCM14, IOMUX_PINCM19, 
                          IOMUX_PINCM20, IOMUX_PINCM25, IOMUX_PINCM26, 
                          IOMUX_PINCM38, IOMUX_PINCM39, IOMUX_PINCM40, 
                          IOMUX_PINCM41, IOMUX_PINCM42, IOMUX_PINCM49, 
                          IOMUX_PINCM50, IOMUX_PINCM51, IOMUX_PINCM52, 
                          IOMUX_PINCM56, IOMUX_PINCM57, IOMUX_PINCM67, 
                          IOMUX_PINCM68, IOMUX_PINCM69, IOMUX_PINCM73, 
                          IOMUX_PINCM74, IOMUX_PINCM3, IOMUX_PINCM4,
                          IOMUX_PINCM5};
#endif //Has GPIOA and GPIOA enabled
#if defined(GPIOB_BASE) && (DRIVER_CONFIG_HAS_GPIOB==1)
uint8_t PORTB_PINCMs[] = {IOMUX_PINCM12, IOMUX_PINCM13, IOMUX_PINCM15, 
                          IOMUX_PINCM16, IOMUX_PINCM17, IOMUX_PINCM18, 
                          IOMUX_PINCM27, IOMUX_PINCM28, IOMUX_PINCM29, 
                          IOMUX_PINCM30, IOMUX_PINCM31, IOMUX_PINCM32, 
                          IOMUX_PINCM33, IOMUX_PINCM34, IOMUX_PINCM35, 
                          IOMUX_PINCM36, IOMUX_PINCM37, IOMUX_PINCM53, 
                          IOMUX_PINCM54, IOMUX_PINCM55, IOMUX_PINCM62, 
                          IOMUX_PINCM63, IOMUX_PINCM64, IOMUX_PINCM65, 
                          IOMUX_PINCM66, IOMUX_PINCM70, IOMUX_PINCM71, 
                          IOMUX_PINCM72, IOMUX_PINCM21, IOMUX_PINCM22,
                          IOMUX_PINCM23, IOMUX_PINCM24};
#endif //Has GPIOB and GPIOB enabled
#if defined(GPIOC_BASE) && (DRIVER_CONFIG_HAS_GPIOC==1)
uint8_t PORTC_PINCMs[] = {IOMUX_PINCM44, IOMUX_PINCM45, IOMUX_PINCM46, 
                          IOMUX_PINCM47, IOMUX_PINCM48, IOMUX_PINCM58, 
                          IOMUX_PINCM59, IOMUX_PINCM60, IOMUX_PINCM61};
#endif //Has GPIOC and GPIOC enabled
#endif //__MSPM0L2228__



/* The GPIOx driver instance expansion generators are given below.
 * These generators are used to create the corresponding data structures and 
 * function linkage implementations that are required by the Arm 
 * CMSIS-Drivers specification for each individual GPIO driver instance used 
 * in an application.
 */

#if defined(GPIOA_BASE) && (DRIVER_CONFIG_HAS_GPIOA==1)
DRIVER_GPIO_MSP_STATE DRIVER_GPIOA_MSP_STATE;
DRIVER_GPIO_MSP DRIVER_GPIOA_MSP =
{
    .hw = GPIOA,
    .state = &DRIVER_GPIOA_MSP_STATE,
    .pinCMs = &PORTA_PINCMs[0],
    .irq = GPIOA_INT_IRQn
};

static int32_t MSP_ARM_GPIOA_Setup(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_SignalEvent_t cb_event) \
{ \
return MSP_ARM_GPIO_Setup(&DRIVER_GPIOA_MSP, pin, cb_event); \
} \
static int32_t MSP_ARM_GPIOA_SetDirection(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_DIRECTION direction) \
{ \
return MSP_ARM_GPIO_SetDirection(&DRIVER_GPIOA_MSP, pin, direction); \
} \
static int32_t MSP_ARM_GPIOA_SetOutputMode(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_OUTPUT_MODE mode) \
{ \
return MSP_ARM_GPIO_SetOutputMode(&DRIVER_GPIOA_MSP, pin, mode); \
} \
static int32_t MSP_ARM_GPIOA_SetPullResistor(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_PULL_RESISTOR resistor) \
{ \
return MSP_ARM_GPIO_SetPullResistor(&DRIVER_GPIOA_MSP, pin, resistor); \
} \
static int32_t MSP_ARM_GPIOA_SetEventTrigger(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_EVENT_TRIGGER trigger) \
{ \
return MSP_ARM_GPIO_SetEventTrigger(&DRIVER_GPIOA_MSP, pin, trigger); \
} \
static void MSP_ARM_GPIOA_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val) \
{ \
MSP_ARM_GPIO_SetOutput(&DRIVER_GPIOA_MSP, pin, val); \
} \
static uint32_t MSP_ARM_GPIOA_GetInput(ARM_GPIO_Pin_t pin) \
{ \
return MSP_ARM_GPIO_GetInput(&DRIVER_GPIOA_MSP, pin); \
} \

ARM_DRIVER_GPIO Driver_GPIOA = \
{ \
MSP_ARM_GPIOA_Setup, \
MSP_ARM_GPIOA_SetDirection, \
MSP_ARM_GPIOA_SetOutputMode, \
MSP_ARM_GPIOA_SetPullResistor, \
MSP_ARM_GPIOA_SetEventTrigger, \
MSP_ARM_GPIOA_SetOutput, \
MSP_ARM_GPIOA_GetInput, \
};
#endif

#if defined(GPIOB_BASE) && (DRIVER_CONFIG_HAS_GPIOB==1)
DRIVER_GPIO_MSP_STATE DRIVER_GPIOB_MSP_STATE;
DRIVER_GPIO_MSP DRIVER_GPIOB_MSP =
{
    .hw = GPIOB,
    .state = &DRIVER_GPIOB_MSP_STATE,
    .pinCMs = &PORTB_PINCMs[0],
    .irq = GPIOB_INT_IRQn
};

static int32_t MSP_ARM_GPIOB_Setup(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_SignalEvent_t cb_event) \
{ \
return MSP_ARM_GPIO_Setup(&DRIVER_GPIOB_MSP, pin, cb_event); \
} \
static int32_t MSP_ARM_GPIOB_SetDirection(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_DIRECTION direction) \
{ \
return MSP_ARM_GPIO_SetDirection(&DRIVER_GPIOB_MSP, pin, direction); \
} \
static int32_t MSP_ARM_GPIOB_SetOutputMode(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_OUTPUT_MODE mode) \
{ \
return MSP_ARM_GPIO_SetOutputMode(&DRIVER_GPIOB_MSP, pin, mode); \
} \
static int32_t MSP_ARM_GPIOB_SetPullResistor(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_PULL_RESISTOR resistor) \
{ \
return MSP_ARM_GPIO_SetPullResistor(&DRIVER_GPIOB_MSP, pin, resistor); \
} \
static int32_t MSP_ARM_GPIOB_SetEventTrigger(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_EVENT_TRIGGER trigger) \
{ \
return MSP_ARM_GPIO_SetEventTrigger(&DRIVER_GPIOB_MSP, pin, trigger); \
} \
static void MSP_ARM_GPIOB_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val) \
{ \
MSP_ARM_GPIO_SetOutput(&DRIVER_GPIOB_MSP, pin, val); \
} \
static uint32_t MSP_ARM_GPIOB_GetInput(ARM_GPIO_Pin_t pin) \
{ \
return MSP_ARM_GPIO_GetInput(&DRIVER_GPIOB_MSP, pin); \
} \

ARM_DRIVER_GPIO Driver_GPIOB = \
{ \
MSP_ARM_GPIOB_Setup, \
MSP_ARM_GPIOB_SetDirection, \
MSP_ARM_GPIOB_SetOutputMode, \
MSP_ARM_GPIOB_SetPullResistor, \
MSP_ARM_GPIOB_SetEventTrigger, \
MSP_ARM_GPIOB_SetOutput, \
MSP_ARM_GPIOB_GetInput, \
};
#endif

#if defined(GPIOC_BASE) && (DRIVER_CONFIG_HAS_GPIOC==1)
DRIVER_GPIO_MSP_STATE DRIVER_GPIOC_MSP_STATE;
DRIVER_GPIO_MSP DRIVER_GPIOC_MSP =
{
    .hw = GPIOC,
    .state = &DRIVER_GPIOC_MSP_STATE,
    .pinCMs = &PORTC_PINCMs[0],
    .irq = GPIOC_INT_IRQn
};

static int32_t MSP_ARM_GPIOC_Setup(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_SignalEvent_t cb_event) \
{ \
return MSP_ARM_GPIO_Setup(&DRIVER_GPIOC_MSP, pin, cb_event); \
} \
static int32_t MSP_ARM_GPIOC_SetDirection(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_DIRECTION direction) \
{ \
return MSP_ARM_GPIO_SetDirection(&DRIVER_GPIOC_MSP, pin, direction); \
} \
static int32_t MSP_ARM_GPIOC_SetOutputMode(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_OUTPUT_MODE mode) \
{ \
return MSP_ARM_GPIO_SetOutputMode(&DRIVER_GPIOC_MSP, pin, mode); \
} \
static int32_t MSP_ARM_GPIOC_SetPullResistor(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_PULL_RESISTOR resistor) \
{ \
return MSP_ARM_GPIO_SetPullResistor(&DRIVER_GPIOC_MSP, pin, resistor); \
} \
static int32_t MSP_ARM_GPIOC_SetEventTrigger(ARM_GPIO_Pin_t pin, \
    ARM_GPIO_EVENT_TRIGGER trigger) \
{ \
return MSP_ARM_GPIO_SetEventTrigger(&DRIVER_GPIOC_MSP, pin, trigger); \
} \
static void MSP_ARM_GPIOC_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val) \
{ \
MSP_ARM_GPIO_SetOutput(&DRIVER_GPIOC_MSP, pin, val); \
} \
static uint32_t MSP_ARM_GPIOC_GetInput(ARM_GPIO_Pin_t pin) \
{ \
return MSP_ARM_GPIO_GetInput(&DRIVER_GPIOC_MSP, pin); \
} \

ARM_DRIVER_GPIO Driver_GPIOC = \
{ \
MSP_ARM_GPIOC_Setup, \
MSP_ARM_GPIOC_SetDirection, \
MSP_ARM_GPIOC_SetOutputMode, \
MSP_ARM_GPIOC_SetPullResistor, \
MSP_ARM_GPIOC_SetEventTrigger, \
MSP_ARM_GPIOC_SetOutput, \
MSP_ARM_GPIOC_GetInput, \
};
#endif

void GROUP1_IRQHandler(void) 
{ 
#if defined(GPIOC_BASE) && (DRIVER_CONFIG_HAS_GPIOC==1)
    MSP_ARM_GPIO_IRQHandler(&DRIVER_GPIOA_MSP); 
#endif
#if defined(GPIOB_BASE) && (DRIVER_CONFIG_HAS_GPIOB==1)
    MSP_ARM_GPIO_IRQHandler(&DRIVER_GPIOB_MSP);
#endif
#if defined(GPIOC_BASE) && (DRIVER_CONFIG_HAS_GPIOC==1)
    MSP_ARM_GPIO_IRQHandler(&DRIVER_GPIOC_MSP); 
#endif
}
