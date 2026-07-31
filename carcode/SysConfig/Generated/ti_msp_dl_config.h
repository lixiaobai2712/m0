/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for MOTOR_PWM */
#define MOTOR_PWM_INST                                                     TIMG0
#define MOTOR_PWM_INST_IRQHandler                               TIMG0_IRQHandler
#define MOTOR_PWM_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define MOTOR_PWM_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_MOTOR_PWM_C0_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C0_PIN                                     DL_GPIO_PIN_12
#define GPIO_MOTOR_PWM_C0_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_MOTOR_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_MOTOR_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MOTOR_PWM_C1_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C1_PIN                                     DL_GPIO_PIN_13
#define GPIO_MOTOR_PWM_C1_IOMUX                                  (IOMUX_PINCM35)
#define GPIO_MOTOR_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_MOTOR_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX




/* Defines for ENCODER_M2 */
#define ENCODER_M2_INST                                                    TIMG8
#define ENCODER_M2_INST_IRQHandler                              TIMG8_IRQHandler
#define ENCODER_M2_INST_INT_IRQN                                (TIMG8_INT_IRQn)
/* Pin configuration defines for ENCODER_M2 PHA Pin */
#define GPIO_ENCODER_M2_PHA_PORT                                           GPIOA
#define GPIO_ENCODER_M2_PHA_PIN                                   DL_GPIO_PIN_26
#define GPIO_ENCODER_M2_PHA_IOMUX                                (IOMUX_PINCM59)
#define GPIO_ENCODER_M2_PHA_IOMUX_FUNC               IOMUX_PINCM59_PF_TIMG8_CCP0
/* Pin configuration defines for ENCODER_M2 PHB Pin */
#define GPIO_ENCODER_M2_PHB_PORT                                           GPIOA
#define GPIO_ENCODER_M2_PHB_PIN                                   DL_GPIO_PIN_27
#define GPIO_ENCODER_M2_PHB_IOMUX                                (IOMUX_PINCM60)
#define GPIO_ENCODER_M2_PHB_IOMUX_FUNC               IOMUX_PINCM60_PF_TIMG8_CCP1



/* Defines for OLED_I2C */
#define OLED_I2C_INST                                                       I2C0
#define OLED_I2C_INST_IRQHandler                                 I2C0_IRQHandler
#define OLED_I2C_INST_INT_IRQN                                     I2C0_INT_IRQn
#define OLED_I2C_BUS_SPEED_HZ                                             400000
#define GPIO_OLED_I2C_SDA_PORT                                             GPIOA
#define GPIO_OLED_I2C_SDA_PIN                                     DL_GPIO_PIN_28
#define GPIO_OLED_I2C_IOMUX_SDA                                   (IOMUX_PINCM3)
#define GPIO_OLED_I2C_IOMUX_SDA_FUNC                    IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_OLED_I2C_SCL_PORT                                             GPIOA
#define GPIO_OLED_I2C_SCL_PIN                                     DL_GPIO_PIN_31
#define GPIO_OLED_I2C_IOMUX_SCL                                   (IOMUX_PINCM6)
#define GPIO_OLED_I2C_IOMUX_SCL_FUNC                    IOMUX_PINCM6_PF_I2C0_SCL


/* Defines for TUNER_UART */
#define TUNER_UART_INST                                                    UART1
#define TUNER_UART_INST_FREQUENCY                                       32000000
#define TUNER_UART_INST_IRQHandler                              UART1_IRQHandler
#define TUNER_UART_INST_INT_IRQN                                  UART1_INT_IRQn
#define GPIO_TUNER_UART_RX_PORT                                            GPIOB
#define GPIO_TUNER_UART_TX_PORT                                            GPIOB
#define GPIO_TUNER_UART_RX_PIN                                     DL_GPIO_PIN_7
#define GPIO_TUNER_UART_TX_PIN                                     DL_GPIO_PIN_6
#define GPIO_TUNER_UART_IOMUX_RX                                 (IOMUX_PINCM24)
#define GPIO_TUNER_UART_IOMUX_TX                                 (IOMUX_PINCM23)
#define GPIO_TUNER_UART_IOMUX_RX_FUNC                  IOMUX_PINCM24_PF_UART1_RX
#define GPIO_TUNER_UART_IOMUX_TX_FUNC                  IOMUX_PINCM23_PF_UART1_TX
#define TUNER_UART_BAUD_RATE                                            (115200)
#define TUNER_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define TUNER_UART_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for CAMERA_UART */
#define CAMERA_UART_INST                                                     UART3
#define CAMERA_UART_INST_FREQUENCY                                        32000000
#define CAMERA_UART_INST_IRQHandler                               UART3_IRQHandler
#define CAMERA_UART_INST_INT_IRQN                                   UART3_INT_IRQn
#define GPIO_CAMERA_UART_RX_PORT                                             GPIOB
#define GPIO_CAMERA_UART_TX_PORT                                             GPIOB
#define GPIO_CAMERA_UART_RX_PIN                                      DL_GPIO_PIN_3
#define GPIO_CAMERA_UART_TX_PIN                                      DL_GPIO_PIN_2
#define GPIO_CAMERA_UART_IOMUX_RX                                  (IOMUX_PINCM20)
#define GPIO_CAMERA_UART_IOMUX_TX                                  (IOMUX_PINCM19)
#define GPIO_CAMERA_UART_IOMUX_RX_FUNC                   IOMUX_PINCM20_PF_UART3_RX
#define GPIO_CAMERA_UART_IOMUX_TX_FUNC                   IOMUX_PINCM19_PF_UART3_TX
#define CAMERA_UART_BAUD_RATE                                             (115200)
#define CAMERA_UART_IBRD_32_MHZ_115200_BAUD                                   (17)
#define CAMERA_UART_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for STEPPER_UART */
#define STEPPER_UART_INST                                                    UART2
#define STEPPER_UART_INST_FREQUENCY                                       32000000
#define STEPPER_UART_INST_IRQHandler                              UART2_IRQHandler
#define STEPPER_UART_INST_INT_IRQN                                  UART2_INT_IRQn
#define GPIO_STEPPER_UART_RX_PORT                                            GPIOA
#define GPIO_STEPPER_UART_TX_PORT                                            GPIOA
#define GPIO_STEPPER_UART_RX_PIN                                     DL_GPIO_PIN_24
#define GPIO_STEPPER_UART_TX_PIN                                     DL_GPIO_PIN_23
#define GPIO_STEPPER_UART_IOMUX_RX                                 (IOMUX_PINCM54)
#define GPIO_STEPPER_UART_IOMUX_TX                                 (IOMUX_PINCM53)
#define GPIO_STEPPER_UART_IOMUX_RX_FUNC                  IOMUX_PINCM54_PF_UART2_RX
#define GPIO_STEPPER_UART_IOMUX_TX_FUNC                  IOMUX_PINCM53_PF_UART2_TX
#define STEPPER_UART_BAUD_RATE                                            (115200)
#define STEPPER_UART_IBRD_32_MHZ_115200_BAUD                                  (17)
#define STEPPER_UART_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for GYRO_UART */
#define GYRO_UART_INST                                                     UART0
#define GYRO_UART_INST_FREQUENCY                                        32000000
#define GYRO_UART_INST_IRQHandler                               UART0_IRQHandler
#define GYRO_UART_INST_INT_IRQN                                   UART0_INT_IRQn
#define GPIO_GYRO_UART_RX_PORT                                             GPIOA
#define GPIO_GYRO_UART_TX_PORT                                             GPIOA
#define GPIO_GYRO_UART_RX_PIN                                      DL_GPIO_PIN_1
#define GPIO_GYRO_UART_TX_PIN                                      DL_GPIO_PIN_0
#define GPIO_GYRO_UART_IOMUX_RX                                   (IOMUX_PINCM2)
#define GPIO_GYRO_UART_IOMUX_TX                                   (IOMUX_PINCM1)
#define GPIO_GYRO_UART_IOMUX_RX_FUNC                    IOMUX_PINCM2_PF_UART0_RX
#define GPIO_GYRO_UART_IOMUX_TX_FUNC                    IOMUX_PINCM1_PF_UART0_TX
#define GYRO_UART_BAUD_RATE                                             (115200)
#define GYRO_UART_IBRD_32_MHZ_115200_BAUD                                   (17)
#define GYRO_UART_FBRD_32_MHZ_115200_BAUD                                   (23)





/* Defines for START_KEY: GPIOB.21 with pinCMx 49 on package pin 20 */
#define CAR_GPIO_START_KEY_PORT                                          (GPIOB)
#define CAR_GPIO_START_KEY_PIN                                  (DL_GPIO_PIN_21)
#define CAR_GPIO_START_KEY_IOMUX                                 (IOMUX_PINCM49)
/* Defines for KEY_K1: GPIOB.13 with pinCMx 30 on package pin 10 */
#define CAR_GPIO_KEY_K1_PORT                                              (GPIOB)
#define CAR_GPIO_KEY_K1_PIN                                      (DL_GPIO_PIN_13)
#define CAR_GPIO_KEY_K1_IOMUX                                    (IOMUX_PINCM30)
/* Defines for KEY_K2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define CAR_GPIO_KEY_K2_PORT                                              (GPIOB)
#define CAR_GPIO_KEY_K2_PIN                                      (DL_GPIO_PIN_23)
#define CAR_GPIO_KEY_K2_IOMUX                                    (IOMUX_PINCM51)
/* Defines for KEY_K3: GPIOB.26 with pinCMx 32 on package pin 25 */
#define CAR_GPIO_KEY_K3_PORT                                              (GPIOB)
#define CAR_GPIO_KEY_K3_PIN                                      (DL_GPIO_PIN_26)
#define CAR_GPIO_KEY_K3_IOMUX                                    (IOMUX_PINCM32)
/* Defines for KEY_K4: GPIOB.27 with pinCMx 33 on package pin 26 */
#define CAR_GPIO_KEY_K4_PORT                                              (GPIOB)
#define CAR_GPIO_KEY_K4_PIN                                      (DL_GPIO_PIN_27)
#define CAR_GPIO_KEY_K4_IOMUX                                    (IOMUX_PINCM33)
/* Defines for AIN1: GPIOB.17 with pinCMx 43 on package pin 14 */
#define CAR_GPIO_AIN1_PORT                                               (GPIOB)
#define CAR_GPIO_AIN1_PIN                                       (DL_GPIO_PIN_17)
#define CAR_GPIO_AIN1_IOMUX                                      (IOMUX_PINCM43)
/* Defines for AIN2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define CAR_GPIO_AIN2_PORT                                               (GPIOB)
#define CAR_GPIO_AIN2_PIN                                       (DL_GPIO_PIN_19)
#define CAR_GPIO_AIN2_IOMUX                                      (IOMUX_PINCM45)
/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define CAR_GPIO_BIN1_PORT                                               (GPIOA)
#define CAR_GPIO_BIN1_PIN                                       (DL_GPIO_PIN_16)
#define CAR_GPIO_BIN1_IOMUX                                      (IOMUX_PINCM38)
/* Defines for BIN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define CAR_GPIO_BIN2_PORT                                               (GPIOB)
#define CAR_GPIO_BIN2_PIN                                       (DL_GPIO_PIN_24)
#define CAR_GPIO_BIN2_IOMUX                                      (IOMUX_PINCM52)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MOTOR_PWM_init(void);
void SYSCFG_DL_ENCODER_M2_init(void);
void SYSCFG_DL_OLED_I2C_init(void);
void SYSCFG_DL_TUNER_UART_init(void);
void SYSCFG_DL_GYRO_UART_init(void);
void SYSCFG_DL_STEPPER_UART_init(void);
void SYSCFG_DL_CAMERA_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
