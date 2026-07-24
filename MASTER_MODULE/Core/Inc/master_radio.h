#ifndef MASTER_RADIO_H
#define MASTER_RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "w5500_port.h"
#include <stdint.h>

#define MASTER_RADIO_NODE_COUNT             4U
#define MASTER_RADIO_EVENT_PAYLOAD_LEN      9U
#define MASTER_RADIO_RESET_PAYLOAD_LEN      9U
#define MASTER_RADIO_SYNC_PAYLOAD_LEN       9U
#define MASTER_RADIO_HEALTH_PAYLOAD_LEN     9U

/* Diagnostic counters for STM32CubeMonitor. */
extern volatile uint32_t g_radio_timestamp_rx_count;
extern volatile uint32_t g_radio_grouped_event_count;
extern volatile uint32_t g_radio_nrf2_irq_count;
extern volatile uint32_t g_radio_nrf3_irq_count;
extern volatile uint32_t g_radio_nrf2_reset_tx_count;
extern volatile uint32_t g_radio_nrf2_sync_tx_count;
extern volatile uint32_t g_radio_nrf2_health_tx_count;
extern volatile uint32_t g_radio_nrf2_tx_ds_count;
extern volatile uint32_t g_radio_nrf2_max_rt_count;
extern volatile uint8_t  g_radio_nrf2_last_tx_cmd;
extern volatile uint8_t  g_radio_nrf2_last_tx_no_ack;
extern volatile uint8_t  g_radio_nrf2_last_payload_spi_cmd;
extern volatile uint8_t  g_radio_nrf2_status_before_tx;
extern volatile uint8_t  g_radio_nrf2_status_after_ce;
extern volatile uint8_t  g_radio_nrf2_tx_addr0;
extern volatile uint8_t  g_radio_nrf2_addr_mode;
extern volatile uint8_t  g_radio_nrf2_rx_p0_addr0;
extern volatile uint8_t  g_radio_health_current_index;
extern volatile uint8_t  g_radio_health_waiting;
extern volatile uint8_t  g_radio_health_cycle_active;
extern volatile uint8_t  g_radio_health_status_0;
extern volatile uint8_t  g_radio_health_status_1;
extern volatile uint8_t  g_radio_health_status_2;
extern volatile uint8_t  g_radio_health_status_3;
extern volatile uint32_t g_radio_nrf2_common_addr_restore_count;
extern volatile uint32_t g_radio_nrf2_sync_deferred_count;
extern volatile uint8_t  g_radio_last_event_id;
extern volatile uint8_t  g_radio_last_pipe;
extern volatile uint8_t  g_radio_health_mask;
extern volatile uint8_t  g_radio_nrf2_present;
extern volatile uint8_t  g_radio_nrf3_present;
extern volatile uint8_t  g_radio_nrf2_last_status;
extern volatile uint8_t  g_radio_nrf2_last_fifo;
extern volatile uint8_t  g_radio_nrf2_config;
extern volatile uint8_t  g_radio_nrf2_observe_tx;
extern volatile uint8_t  g_radio_nrf2_rf_ch_read;
extern volatile uint8_t  g_radio_nrf2_setup_aw_read;
extern volatile uint8_t  g_radio_nrf2_status_nop;
extern volatile uint8_t  g_radio_nrf2_present_code;
extern volatile uint8_t  g_radio_nrf2_irq_pin_level;
extern volatile uint32_t g_radio_nrf2_spi_error_count;
extern volatile uint8_t  g_radio_nrf2_status_after_clear;
extern volatile uint8_t  g_radio_nrf2_feature;
extern volatile uint8_t  g_radio_nrf2_dynpd;
extern volatile uint8_t  g_radio_nrf3_last_status;
extern volatile uint8_t  g_radio_nrf3_last_fifo;
extern volatile uint8_t  g_radio_nrf3_config;
extern volatile uint8_t  g_radio_nrf3_observe_tx;
extern volatile uint8_t  g_radio_nrf3_rf_ch_read;
extern volatile uint8_t  g_radio_nrf3_setup_aw_read;
extern volatile uint8_t  g_radio_nrf3_status_nop;
extern volatile uint8_t  g_radio_nrf3_present_code;
extern volatile uint8_t  g_radio_nrf3_irq_pin_level;
extern volatile uint32_t g_radio_nrf3_spi_error_count;
extern volatile uint8_t  g_radio_nrf3_status_after_clear;
extern volatile uint8_t  g_radio_nrf3_feature;
extern volatile uint8_t  g_radio_nrf3_dynpd;
extern volatile uint8_t  g_radio_nrf3_rx_dr_seen;
extern volatile uint8_t  g_radio_nrf3_poll_status;
extern volatile uint32_t g_radio_nrf3_poll_rx_dr_count;
extern volatile uint32_t g_radio_nrf3_bad_pipe_count;
extern volatile uint32_t g_radio_nrf3_flush_count;
extern volatile uint32_t g_master_tim2_overflows;

void MasterRadio_Init(void);
void MasterRadio_Service(void);
void MasterRadio_OnExti(uint16_t GPIO_Pin);
void MasterRadio_OnTimerPeriodElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_RADIO_H */