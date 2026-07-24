/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint64_t sample_index;
    uint64_t timestamp_us;
    uint32_t peak_envelope;
    uint32_t peak_sta_lta_q8;
    uint32_t noise_mean;
    uint32_t noise_deviation;
    uint16_t dwell_samples;
    uint8_t  quality_flags;
    uint8_t  valid;
} transient_event_t;

typedef struct __attribute__((packed)) {
    uint64_t timestamp_sample;  /* В текущей логике сюда допускается запись ev.timestamp_us. */
    uint8_t  event_id;
} radio_event_packet_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile transient_event_t last_event;
extern volatile uint64_t last_event_timestamp_us;
extern volatile uint64_t last_event_sample_index;
extern volatile uint32_t dma_audio_overruns;
extern volatile uint8_t recognized_event_id;
extern volatile uint8_t radio_event_ready;
extern volatile radio_event_packet_t radio_event_packet;
extern volatile uint8_t SynEnable;
extern volatile int32_t radio_last_sync_error_us;
extern volatile uint32_t radio_nrf3_max_rt_count;
extern volatile uint32_t radio_nrf3_tx_ok_count;
extern volatile uint32_t radio_nrf2_sync_count;
extern volatile uint32_t radio_nrf2_reset_count;
extern volatile uint32_t radio_nrf2_health_count;
extern volatile uint8_t radio_nrf2_irq_pending;
extern volatile uint8_t radio_nrf3_irq_pending;
extern volatile uint8_t radio_nrf3_tx_busy;
extern volatile uint8_t radio_nrf2_present;
extern volatile uint8_t radio_nrf3_present;
extern volatile uint8_t radio_nrf2_config;
extern volatile uint8_t radio_nrf3_config;
extern volatile uint8_t radio_nrf2_last_status;
extern volatile uint8_t radio_nrf3_last_status;
extern volatile uint8_t radio_nrf2_fifo_status;
extern volatile uint8_t radio_nrf3_fifo_status;
extern volatile uint8_t radio_nrf2_observe_tx;
extern volatile uint8_t radio_nrf3_observe_tx;
extern volatile uint8_t radio_nrf2_rf_ch_read;
extern volatile uint8_t radio_nrf3_rf_ch_read;
extern volatile uint8_t radio_nrf2_setup_aw_read;
extern volatile uint8_t radio_nrf3_setup_aw_read;
extern volatile uint8_t radio_nrf2_irq_pin_level;
extern volatile uint8_t radio_nrf3_irq_pin_level;
extern volatile uint8_t radio_nrf2_status_after_clear;
extern volatile uint8_t radio_nrf3_status_after_clear;
extern volatile uint32_t radio_nrf2_init_clear_count;
extern volatile uint32_t radio_nrf3_init_clear_count;
extern volatile uint32_t radio_nrf2_spi_error_count;
extern volatile uint32_t radio_nrf3_spi_error_count;
extern volatile uint8_t radio_nrf2_status_nop;
extern volatile uint8_t radio_nrf3_status_nop;
extern volatile uint8_t radio_nrf2_present_code;
extern volatile uint8_t radio_nrf3_present_code;
extern volatile uint8_t radio_nrf2_feature;
extern volatile uint8_t radio_nrf3_feature;
extern volatile uint8_t radio_nrf2_dynpd;
extern volatile uint8_t radio_nrf3_dynpd;
extern volatile uint8_t radio_nrf2_en_aa;
extern volatile uint32_t radio_nrf2_last_sync_master_us_lo;
extern volatile uint32_t radio_nrf2_last_sync_master_us_hi;
void Radio_IRQ_Handler(uint16_t GPIO_Pin);
void Radio_ProcessMainLoop(void);
void Radio_PublishEventTimestamp(uint64_t timestamp_us);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define NRF2_CSN_Pin GPIO_PIN_12
#define NRF2_CSN_GPIO_Port GPIOB
#define NRF2_IRQ_Pin GPIO_PIN_8
#define NRF2_IRQ_GPIO_Port GPIOA
#define NRF2_CE_Pin GPIO_PIN_9
#define NRF2_CE_GPIO_Port GPIOA
#define NRF3_IRQ_Pin GPIO_PIN_11
#define NRF3_IRQ_GPIO_Port GPIOA
#define NRF3_CE_Pin GPIO_PIN_12
#define NRF3_CE_GPIO_Port GPIOA
#define NRF3_CSN_Pin GPIO_PIN_15
#define NRF3_CSN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* nRF24L01+ register addresses */
#define NRF_REG_CONFIG       0x00U
#define NRF_REG_EN_AA        0x01U
#define NRF_REG_EN_RXADDR    0x02U
#define NRF_REG_SETUP_AW     0x03U
#define NRF_REG_SETUP_RETR   0x04U
#define NRF_REG_RF_CH        0x05U
#define NRF_REG_RF_SETUP     0x06U
#define NRF_REG_STATUS       0x07U
#define NRF_REG_OBSERVE_TX   0x08U
#define NRF_REG_RPD          0x09U
#define NRF_REG_RX_ADDR_P0   0x0AU
#define NRF_REG_RX_ADDR_P1   0x0BU
#define NRF_REG_RX_ADDR_P2   0x0CU
#define NRF_REG_RX_ADDR_P3   0x0DU
#define NRF_REG_RX_ADDR_P4   0x0EU
#define NRF_REG_RX_ADDR_P5   0x0FU
#define NRF_REG_TX_ADDR      0x10U
#define NRF_REG_RX_PW_P0     0x11U
#define NRF_REG_RX_PW_P1     0x12U
#define NRF_REG_RX_PW_P2     0x13U
#define NRF_REG_RX_PW_P3     0x14U
#define NRF_REG_RX_PW_P4     0x15U
#define NRF_REG_RX_PW_P5     0x16U
#define NRF_REG_FIFO_STATUS  0x17U
#define NRF_REG_DYNPD        0x1CU
#define NRF_REG_FEATURE      0x1DU

/* nRF24L01+ SPI commands */
#define NRF_CMD_R_REGISTER        0x00U
#define NRF_CMD_W_REGISTER        0x20U
#define NRF_CMD_R_RX_PAYLOAD      0x61U
#define NRF_CMD_W_TX_PAYLOAD      0xA0U
#define NRF_CMD_FLUSH_TX          0xE1U
#define NRF_CMD_FLUSH_RX          0xE2U
#define NRF_CMD_REUSE_TX_PL       0xE3U
#define NRF_CMD_R_RX_PL_WID       0x60U
#define NRF_CMD_W_ACK_PAYLOAD     0xA8U
#define NRF_CMD_W_TX_PAYLOAD_NOACK 0xB0U
#define NRF_CMD_NOP               0xFFU

/* STATUS bits */
#define NRF_STATUS_RX_DR      0x40U
#define NRF_STATUS_TX_DS      0x20U
#define NRF_STATUS_MAX_RT     0x10U
#define NRF_STATUS_RX_P_NO    0x0EU

/* Packet markers for NRF2 receiver. Values must match master firmware. */
#define RADIO_PKT_TIME_RESET  0xA0U
#define RADIO_PKT_SYNC        0xA1U
#define RADIO_PKT_HEALTH      0xA2U

#define RADIO_ADDR_WIDTH      5U
#define RADIO_EVENT_PAYLOAD_LEN ((uint8_t)sizeof(radio_event_packet_t))
#define RADIO_CTRL_PAYLOAD_LEN  9U

/* RF channels must be different so NRF2 and NRF3 do not interfere. */
#define NRF2_FREQ             40U
#define NRF3_FREQ             76U

/* Sync settings. Master must use the same period. */
#define RADIO_SYNC_INTERVAL_US       1048576ULL
#define RADIO_SYNC_CALIBRATION_US    190L
#define RADIO_SYNC_MAX_ERROR_US      2617L
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */