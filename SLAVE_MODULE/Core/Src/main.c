/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    DET_STATE_QUIET = 0,
    DET_STATE_CANDIDATE,
    DET_STATE_LOCKOUT
} detector_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_FS             16000U
#define BLOCK_SAMPLES        512U
#define DMA_HALVES           2U
/* INMP441 передает полезный моно-сигнал в одном слоте стереокадра I2S.
 * Один 24-битный слот занимает два 16-битных DMA half-word.
 * Один полный стереокадр: левый слот + правый слот = 4 half-word.
 * Поэтому один полезный моно-отсчет соответствует 4 half-word, а не 2.
 */
#define I2S_WORDS_PER_SAMPLE      4U
#define I2S_ACTIVE_SLOT_WORD_OFFSET 0U  /* L/R=0: левый слот; если сигнал нулевой, заменить на 2U */
#define I2S_HALF_WORDS            (BLOCK_SAMPLES * I2S_WORDS_PER_SAMPLE)
#define I2S_DMA_WORDS             (BLOCK_SAMPLES * DMA_HALVES * I2S_WORDS_PER_SAMPLE)
/* Для STM32F4 HAL в 24-bit I2S Size удваивается внутри HAL до количества 16-bit DMA-транзакций. */
#define I2S_HAL_RX_SIZE_PARAM     I2S_HALF_WORDS

#define PRETRIGGER_SAMPLES  256U
#define PRETRIGGER_MASK     (PRETRIGGER_SAMPLES - 1U)
#define STA_SAMPLES         8U
#define LTA_SAMPLES         128U
#define STA_LTA_RATIO_Q8    768U
#define TRIGGER_MULTIPLIER  4U
#define ONSET_MULTIPLIER    2U
#define MIN_ONSET_DWELL     4U
#define MIN_SHAPE_DWELL     8U
#define LOCKOUT_SAMPLES     800U
#define CANDIDATE_TIMEOUT   320U
#define DC_SHIFT            10U
#define ENV_SHIFT           2U
#define NOISE_SHIFT         6U
#define CUSUM_LEAK          96U
#define CUSUM_CONFIRM       24000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
DMA_HandleTypeDef hdma_spi1_rx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csn_port;
    uint16_t csn_pin;
    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;
} nrf_dev_t;

volatile transient_event_t last_event = {0};
volatile uint64_t last_event_timestamp_us = 0;
volatile uint64_t last_event_sample_index = 0;
volatile uint32_t dma_audio_overruns = 0;
volatile uint8_t recognized_event_id = 0;
volatile uint8_t radio_event_ready = 0;
volatile radio_event_packet_t radio_event_packet = {0};

/* Диагностика радиоканала для STM32CubeMonitor. */
volatile uint8_t SynEnable = 0;
volatile int32_t radio_last_sync_error_us = 0;
volatile int32_t radio_time_correction_us = 0;
volatile uint8_t radio_sync_seen = 0;
volatile uint8_t radio_nrf3_tx_busy = 0;
volatile uint8_t radio_nrf3_last_status = 0;
volatile uint8_t radio_nrf2_last_status = 0;
volatile uint8_t radio_nrf2_last_pipe = 0;
volatile uint8_t radio_nrf2_last_payload = 0;
volatile uint64_t radio_nrf2_last_sync_master_us = 0;
volatile uint32_t radio_nrf2_last_sync_master_us_lo = 0;
volatile uint32_t radio_nrf2_last_sync_master_us_hi = 0;
volatile uint8_t radio_nrf2_last_payload_len = 0;
volatile uint8_t radio_nrf2_en_aa = 0xFFU;
volatile uint32_t radio_nrf3_max_rt_count = 0;
volatile uint32_t radio_nrf3_tx_ok_count = 0;
volatile uint32_t radio_nrf2_sync_count = 0;
volatile uint32_t radio_nrf2_reset_count = 0;
volatile uint32_t radio_nrf2_health_count = 0;
volatile uint32_t radio_nrf2_lost_sync_estimate = 0;

/* Расширенная диагностика nRF24L01+ для CubeMonitor.
 * present = 1, если по SPI читается RF_CH, равный ожидаемой частоте.
 * irq_pin_level: 0 означает активное IRQ, 1 - линия отпущена.
 * *_status_after_clear должен иметь биты RX_DR/TX_DS/MAX_RT сброшенными. */
volatile uint8_t radio_nrf2_present = 0;
volatile uint8_t radio_nrf3_present = 0;
volatile uint8_t radio_nrf2_config = 0;
volatile uint8_t radio_nrf3_config = 0;
volatile uint8_t radio_nrf2_fifo_status = 0;
volatile uint8_t radio_nrf3_fifo_status = 0;
volatile uint8_t radio_nrf2_observe_tx = 0;
volatile uint8_t radio_nrf3_observe_tx = 0;
volatile uint8_t radio_nrf2_rf_ch_read = 0;
volatile uint8_t radio_nrf3_rf_ch_read = 0;
volatile uint8_t radio_nrf2_setup_aw_read = 0;
volatile uint8_t radio_nrf3_setup_aw_read = 0;
volatile uint8_t radio_nrf2_irq_pin_level = 1;
volatile uint8_t radio_nrf3_irq_pin_level = 1;
volatile uint8_t radio_nrf2_status_after_clear = 0;
volatile uint8_t radio_nrf3_status_after_clear = 0;
volatile uint32_t radio_nrf2_init_clear_count = 0;
volatile uint32_t radio_nrf3_init_clear_count = 0;
volatile uint32_t radio_nrf2_spi_error_count = 0;
volatile uint32_t radio_nrf3_spi_error_count = 0;
volatile uint8_t radio_nrf2_status_nop = 0;
volatile uint8_t radio_nrf3_status_nop = 0;
volatile uint8_t radio_nrf2_present_code = 0;
volatile uint8_t radio_nrf3_present_code = 0;
volatile uint8_t radio_nrf2_feature = 0xFFU;
volatile uint8_t radio_nrf3_feature = 0xFFU;
volatile uint8_t radio_nrf2_dynpd = 0xFFU;
volatile uint8_t radio_nrf3_dynpd = 0xFFU;

/* EXTI must be short: only post these flags. SPI service is done in main loop. */
volatile uint8_t radio_nrf2_irq_pending = 0;
volatile uint8_t radio_nrf3_irq_pending = 0;

/* Адреса должны совпадать с настройками мастера.
 * NRF3 передаёт timestamp-пакеты на pipe мастера, выделенную данному slave.
 * NRF2 принимает pipe0 общий sync/reset и pipe1 индивидуальный health-check. */
/* Адреса согласованы с master_radio_impl_v2/master_radio.c.
 * Для Slave_0:
 *   NRF3 timestamp TX -> master NRF3 RX pipe0: 00 CC BB AA E1
 *   NRF2 common RX    <- master reset/sync:      D2 C2 B2 A2 E2
 *   NRF2 health RX    <- master health slave0:   10 C3 B3 A3 E3
 * Для Slave_1..3 нужно изменить только первый байт nrf3_master_pipe0_addr
 * на 0x01..0x03 и первый байт nrf2_slave_pipe1_addr на 0x11..0x13.
 */
static uint8_t nrf3_master_pipe0_addr[RADIO_ADDR_WIDTH] = {0x00U, 0xCCU, 0xBBU, 0xAAU, 0xE1U};
static uint8_t nrf2_common_pipe0_addr[RADIO_ADDR_WIDTH] = {0xD2U, 0xC2U, 0xB2U, 0xA2U, 0xE2U};
static uint8_t nrf2_slave_pipe1_addr[RADIO_ADDR_WIDTH]  = {0x10U, 0xC3U, 0xB3U, 0xA3U, 0xE3U};

static const nrf_dev_t nrf2_rx = {
    .hspi = &hspi2,
    .csn_port = NRF2_CSN_GPIO_Port,
    .csn_pin = NRF2_CSN_Pin,
    .ce_port = NRF2_CE_GPIO_Port,
    .ce_pin = NRF2_CE_Pin
};
static const nrf_dev_t nrf3_tx = {
    .hspi = &hspi3,
    .csn_port = NRF3_CSN_GPIO_Port,
    .csn_pin = NRF3_CSN_Pin,
    .ce_port = NRF3_CE_GPIO_Port,
    .ce_pin = NRF3_CE_Pin
};

/*
 * 24-битный I2S через 16-битный DMA.
 * Каждый сэмпл = 2 полуслова (half-words) по 16 бит.
 * Двойной буфер: 2 половины * BLOCK_SAMPLES сэмплов * 2 полуслова = BLOCK_SAMPLES * 4
 */
static uint16_t i2s_buffer[I2S_DMA_WORDS] __attribute__((aligned(4)));

/* ISR only posts completed DMA halves; main loop consumes them atomically. */
static volatile uint8_t process_pending_mask = 0;
static volatile uint64_t process_block_start_sample[DMA_HALVES] = {0, 0};

volatile uint32_t tim2_ovf = 0;
static uint64_t recording_start_us = 0;
static volatile uint64_t sample_counter = 0;

static int16_t  pretrigger_pcm[PRETRIGGER_SAMPLES];
static uint16_t pretrigger_env[PRETRIGGER_SAMPLES];
static uint32_t pretrigger_wr = 0;

static detector_state_t det_state = DET_STATE_QUIET;
static int32_t  dc_estimate = 0;
static uint32_t env_state = 0;
static uint32_t noise_mean = 512U;
static uint32_t noise_deviation = 64U;
static uint32_t sta_ring[STA_SAMPLES];
static uint32_t lta_ring[LTA_SAMPLES];
static uint32_t sta_sum = 0;
static uint32_t lta_sum = 1U;
static uint32_t sta_pos = 0;
static uint32_t lta_pos = 0;
static uint32_t cusum_pos = 0;
static uint32_t lockout_left = 0;
static uint32_t candidate_age = 0;
static uint32_t candidate_peak = 0;
static uint32_t candidate_peak_sta_lta_q8 = 0;
static uint16_t candidate_dwell = 0;
static uint64_t candidate_start_sample = 0;

int32_t test24 = 0;

/* Debug-переменные детектора для STM32CubeMonitor.
 * Используются только для построения графиков: выпрямленный сигнал,
 * огибающая, адаптивные пороги и состояние автомата детектора.
 */
volatile uint16_t dbg_rect16 = 0;
volatile uint32_t dbg_env = 0;
volatile uint32_t dbg_thr_trigger = 0;
volatile uint32_t dbg_thr_onset = 0;
volatile uint8_t  dbg_det_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint64_t get_time_us(void);
void process_audio_block(uint16_t *buf, uint16_t len, uint64_t block_start_sample);
static void Radio_Init(void);
static void Radio_ResetInternalTimeAndEvents(void);
static uint64_t Radio_ApplySyncCorrection(uint64_t timestamp_us);
static void Radio_HandleNrf2Rx(void);
static void Radio_HandleNrf3TxIrq(void);
static void nrf_read_payload_buf(const nrf_dev_t *dev, uint8_t *payload, uint8_t len);
static uint64_t get_u64_le(const uint8_t *p);
static void nrf_clear_all_irqs_and_fifos(const nrf_dev_t *dev, uint8_t is_nrf3);
static void Radio_UpdateNrfDebugSnapshot(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t nrf_spi_xfer(const nrf_dev_t *dev, uint8_t data)
{
    uint8_t rx = 0xFFU;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &data, &rx, 1U, 2U);
    if (st != HAL_OK) {
        if (dev == &nrf2_rx) {
            radio_nrf2_spi_error_count++;
        } else if (dev == &nrf3_tx) {
            radio_nrf3_spi_error_count++;
        }
    }
    return rx;
}

static void nrf_csn_low(const nrf_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_RESET);
}

static void nrf_csn_high(const nrf_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_SET);
}

static void nrf_ce_low(const nrf_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_RESET);
}

static void nrf_ce_high(const nrf_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_SET);
}

static uint8_t nrf_read_reg(const nrf_dev_t *dev, uint8_t reg)
{
    uint8_t val;
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, NRF_CMD_R_REGISTER | (reg & 0x1FU));
    val = nrf_spi_xfer(dev, 0x00U);
    nrf_csn_high(dev);
    return val;
}

static void nrf_write_reg(const nrf_dev_t *dev, uint8_t reg, uint8_t val)
{
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, NRF_CMD_W_REGISTER | (reg & 0x1FU));
    nrf_spi_xfer(dev, val);
    nrf_csn_high(dev);
}

static void nrf_read_multi(const nrf_dev_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, NRF_CMD_R_REGISTER | (reg & 0x1FU));
    for (uint8_t i = 0; i < len; i++) {
        data[i] = nrf_spi_xfer(dev, 0x00U);
    }
    nrf_csn_high(dev);
}

static void nrf_write_multi(const nrf_dev_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, NRF_CMD_W_REGISTER | (reg & 0x1FU));
    for (uint8_t i = 0; i < len; i++) {
        nrf_spi_xfer(dev, data[i]);
    }
    nrf_csn_high(dev);
}

static void nrf_command(const nrf_dev_t *dev, uint8_t cmd)
{
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, cmd);
    nrf_csn_high(dev);
}

static void nrf_activate_features(const nrf_dev_t *dev)
{
    /* nRF24L01+ FEATURE/DYNPD registers are locked after reset on many
     * modules. ACTIVATE 0x73 unlocks them. Without it, EN_DYN_ACK can stay
     * zero and NO_ACK sync/reset packets from the master may be ignored. */
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, 0x50U);
    nrf_spi_xfer(dev, 0x73U);
    nrf_csn_high(dev);
}

static void nrf_write_feature_checked(const nrf_dev_t *dev, uint8_t value)
{
    /* ACTIVATE 0x73 toggles FEATURE access. If the MCU resets but the radio
     * module remains powered, unconditional ACTIVATE may switch FEATURE access
     * off. Write-read first; ACTIVATE only if FEATURE did not accept value. */
    nrf_write_reg(dev, NRF_REG_FEATURE, value);
    if (nrf_read_reg(dev, NRF_REG_FEATURE) != value) {
        nrf_activate_features(dev);
        nrf_write_reg(dev, NRF_REG_FEATURE, value);
    }
}


static uint8_t nrf_get_status_nop(const nrf_dev_t *dev)
{
    uint8_t st;
    nrf_csn_low(dev);
    st = nrf_spi_xfer(dev, NRF_CMD_NOP);
    nrf_csn_high(dev);
    return st;
}

static void nrf_clear_irq(const nrf_dev_t *dev, uint8_t flags)
{
    nrf_write_reg(dev, NRF_REG_STATUS, flags & (NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT));
}

static void nrf_clear_all_irqs_and_fifos(const nrf_dev_t *dev, uint8_t is_nrf3)
{
    /* nRF24L01+ IRQ is active-low while RX_DR, TX_DS or MAX_RT is set and
     * unmasked by CONFIG. On power-up or after a failed TX these flags may
     * remain latched. Clear them before enabling CE and before enabling
     * unmasked IRQ sources, otherwise the IRQ line can stay low forever. */
    nrf_ce_low(dev);
    nrf_write_reg(dev, NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    nrf_command(dev, NRF_CMD_FLUSH_RX);
    nrf_command(dev, NRF_CMD_FLUSH_TX);
    nrf_write_reg(dev, NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    if (is_nrf3 != 0U) {
        radio_nrf3_init_clear_count++;
        radio_nrf3_status_after_clear = nrf_read_reg(dev, NRF_REG_STATUS);
        radio_nrf3_fifo_status = nrf_read_reg(dev, NRF_REG_FIFO_STATUS);
    } else {
        radio_nrf2_init_clear_count++;
        radio_nrf2_status_after_clear = nrf_read_reg(dev, NRF_REG_STATUS);
        radio_nrf2_fifo_status = nrf_read_reg(dev, NRF_REG_FIFO_STATUS);
    }
}

static void Radio_UpdateNrfDebugSnapshot(void)
{
    radio_nrf2_status_nop = nrf_get_status_nop(&nrf2_rx);
    radio_nrf3_status_nop = nrf_get_status_nop(&nrf3_tx);
    radio_nrf2_last_status = nrf_read_reg(&nrf2_rx, NRF_REG_STATUS);
    radio_nrf3_last_status = nrf_read_reg(&nrf3_tx, NRF_REG_STATUS);
    radio_nrf2_config = nrf_read_reg(&nrf2_rx, NRF_REG_CONFIG);
    radio_nrf3_config = nrf_read_reg(&nrf3_tx, NRF_REG_CONFIG);
    radio_nrf2_fifo_status = nrf_read_reg(&nrf2_rx, NRF_REG_FIFO_STATUS);
    radio_nrf3_fifo_status = nrf_read_reg(&nrf3_tx, NRF_REG_FIFO_STATUS);
    radio_nrf2_observe_tx = nrf_read_reg(&nrf2_rx, NRF_REG_OBSERVE_TX);
    radio_nrf3_observe_tx = nrf_read_reg(&nrf3_tx, NRF_REG_OBSERVE_TX);
    radio_nrf2_rf_ch_read = nrf_read_reg(&nrf2_rx, NRF_REG_RF_CH);
    radio_nrf3_rf_ch_read = nrf_read_reg(&nrf3_tx, NRF_REG_RF_CH);
    radio_nrf2_setup_aw_read = nrf_read_reg(&nrf2_rx, NRF_REG_SETUP_AW);
    radio_nrf3_setup_aw_read = nrf_read_reg(&nrf3_tx, NRF_REG_SETUP_AW);
    radio_nrf2_feature = nrf_read_reg(&nrf2_rx, NRF_REG_FEATURE);
    radio_nrf3_feature = nrf_read_reg(&nrf3_tx, NRF_REG_FEATURE);
    radio_nrf2_en_aa = nrf_read_reg(&nrf2_rx, NRF_REG_EN_AA);
    radio_nrf2_dynpd = nrf_read_reg(&nrf2_rx, NRF_REG_DYNPD);
    radio_nrf3_dynpd = nrf_read_reg(&nrf3_tx, NRF_REG_DYNPD);
    radio_nrf2_irq_pin_level = (uint8_t)HAL_GPIO_ReadPin(NRF2_IRQ_GPIO_Port, NRF2_IRQ_Pin);
    radio_nrf3_irq_pin_level = (uint8_t)HAL_GPIO_ReadPin(NRF3_IRQ_GPIO_Port, NRF3_IRQ_Pin);
    radio_nrf2_present_code = 0U;
    if (radio_nrf2_rf_ch_read == NRF2_FREQ) { radio_nrf2_present_code |= 0x01U; }
    if (radio_nrf2_setup_aw_read == 0x03U) { radio_nrf2_present_code |= 0x02U; }
    if ((radio_nrf2_status_nop != 0xFFU) && (radio_nrf2_status_nop != 0x00U)) { radio_nrf2_present_code |= 0x04U; }
    radio_nrf2_present = ((radio_nrf2_present_code & 0x03U) == 0x03U) ? 1U : 0U;

    radio_nrf3_present_code = 0U;
    if (radio_nrf3_rf_ch_read == NRF3_FREQ) { radio_nrf3_present_code |= 0x01U; }
    if (radio_nrf3_setup_aw_read == 0x03U) { radio_nrf3_present_code |= 0x02U; }
    if ((radio_nrf3_status_nop != 0xFFU) && (radio_nrf3_status_nop != 0x00U)) { radio_nrf3_present_code |= 0x04U; }
    radio_nrf3_present = ((radio_nrf3_present_code & 0x03U) == 0x03U) ? 1U : 0U;
}

static void delay_us_blocking(uint32_t us)
{
    uint64_t start = get_time_us();
    while ((uint64_t)(get_time_us() - start) < us) {
        __NOP();
    }
}

uint64_t get_time_us(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint64_t t = ((uint64_t)tim2_ovf << 32) | TIM2->CNT;
    if (primask == 0U) {
        __enable_irq();
    }
    return t;
}

static void set_time_us(uint64_t us)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    tim2_ovf = (uint32_t)(us >> 32);
    TIM2->CNT = (uint32_t)us;
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint8_t pop_dma_half(uint16_t **block_ptr, uint64_t *block_start_sample)
{
    uint8_t half_index = 0xFFU;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (process_pending_mask & 0x01U) {
        process_pending_mask &= (uint8_t)~0x01U;
        half_index = 0U;
        *block_start_sample = process_block_start_sample[0];
    } else if (process_pending_mask & 0x02U) {
        process_pending_mask &= (uint8_t)~0x02U;
        half_index = 1U;
        *block_start_sample = process_block_start_sample[1];
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (half_index != 0xFFU) {
        *block_ptr = &i2s_buffer[(uint32_t)half_index * I2S_HALF_WORDS];
    }
    return half_index;
}

static uint8_t audio_has_pending_block(void)
{
    return process_pending_mask != 0U;
}

static void mark_dma_half_ready(uint8_t half_index)
{
    uint8_t bit = (uint8_t)(1U << half_index);
    uint64_t start_sample = sample_counter;

    process_block_start_sample[half_index] = start_sample;
    sample_counter = start_sample + BLOCK_SAMPLES;

    if (process_pending_mask & bit) {
        dma_audio_overruns++;
    }
    process_pending_mask |= bit;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        tim2_ovf++;
    }
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI1) {
        mark_dma_half_ready(0U);
    }
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI1) {
        mark_dma_half_ready(1U);
    }
}

static inline uint32_t u32_abs_i32(int32_t v)
{
    return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

static inline int32_t decode_inmp441_sample24(const uint16_t *buf, uint16_t i)
{
    uint32_t base = (uint32_t)i * I2S_WORDS_PER_SAMPLE + I2S_ACTIVE_SLOT_WORD_OFFSET;
    uint32_t raw24 = ((uint32_t)buf[base] << 8) |
                     ((buf[base + 1U] >> 8) & 0xFFU);

    if (raw24 & 0x800000U) {
        return (int32_t)(raw24 | 0xFF000000U);
    }
    return (int32_t)raw24;
}

static uint32_t detector_threshold(uint32_t multiplier)
{
    uint32_t thr = noise_mean + noise_deviation * multiplier;
    return (thr < 32U) ? 32U : thr;
}

static uint32_t sta_lta_ratio_q8(void)
{
    uint32_t lta = (lta_sum == 0U) ? 1U : lta_sum;
    return (uint32_t)(((uint64_t)sta_sum * LTA_SAMPLES * 256ULL) /
                      ((uint64_t)lta * STA_SAMPLES));
}

static uint8_t sta_lta_confirmed(void)
{
    uint32_t lta = (lta_sum == 0U) ? 1U : lta_sum;
    return (((uint64_t)sta_sum * LTA_SAMPLES * 256ULL) >
            ((uint64_t)lta * STA_SAMPLES * STA_LTA_RATIO_Q8));
}

static void update_sta_lta(uint32_t env)
{
    sta_sum -= sta_ring[sta_pos];
    sta_ring[sta_pos] = env;
    sta_sum += env;
    if (++sta_pos >= STA_SAMPLES) {
        sta_pos = 0U;
    }

    lta_sum -= lta_ring[lta_pos];
    lta_ring[lta_pos] = env;
    lta_sum += env;
    if (++lta_pos >= LTA_SAMPLES) {
        lta_pos = 0U;
    }
}

static void update_noise_if_quiet(uint32_t env)
{
    if (det_state != DET_STATE_QUIET || env > detector_threshold(TRIGGER_MULTIPLIER)) {
        return;
    }

    int32_t mean_delta = (int32_t)env - (int32_t)noise_mean;
    int32_t new_mean = (int32_t)noise_mean + (mean_delta >> NOISE_SHIFT);
    noise_mean = (new_mean < 0) ? 0U : (uint32_t)new_mean;

    uint32_t abs_delta = u32_abs_i32((int32_t)env - (int32_t)noise_mean);
    int32_t dev_delta = (int32_t)abs_delta - (int32_t)noise_deviation;
    int32_t new_dev = (int32_t)noise_deviation + (dev_delta >> NOISE_SHIFT);
    noise_deviation = (new_dev < 8) ? 8U : (uint32_t)new_dev;
}

static void update_cusum(uint32_t env)
{
    uint32_t baseline = detector_threshold(1U);

    if (env > baseline) {
        uint32_t inc = env - baseline;
        if (inc > 4096U) {
            inc = 4096U;
        }
        cusum_pos += inc;
        if (cusum_pos > (CUSUM_CONFIRM * 4U)) {
            cusum_pos = CUSUM_CONFIRM * 4U;
        }
    } else if (cusum_pos > CUSUM_LEAK) {
        cusum_pos -= CUSUM_LEAK;
    } else {
        cusum_pos = 0U;
    }
}

static uint64_t refine_first_arrival(uint64_t current_sample)
{
    uint32_t onset_thr = detector_threshold(ONSET_MULTIPLIER);

    for (uint32_t back = PRETRIGGER_SAMPLES - MIN_ONSET_DWELL - 1U;
         back > MIN_ONSET_DWELL;
         back--) {
        uint32_t idx = (pretrigger_wr - back) & PRETRIGGER_MASK;
        uint32_t ok = 1U;

        for (uint32_t k = 0; k < MIN_ONSET_DWELL; k++) {
            if (pretrigger_env[(idx + k) & PRETRIGGER_MASK] < onset_thr) {
                ok = 0U;
                break;
            }
        }

        if (ok) {
            uint32_t prev = (idx - 1U) & PRETRIGGER_MASK;
            if (pretrigger_env[idx] >= pretrigger_env[prev]) {
                return current_sample - back;
            }
        }
    }
    return current_sample;
}

static void publish_event(uint64_t sample_index,
                          uint32_t peak_env,
                          uint32_t peak_sta_lta_q8,
                          uint16_t dwell)
{
    transient_event_t ev;

    ev.sample_index = sample_index;
    ev.timestamp_us = recording_start_us + ((sample_index * 1000000ULL) / AUDIO_FS);
    ev.peak_envelope = peak_env;
    ev.peak_sta_lta_q8 = peak_sta_lta_q8;
    ev.noise_mean = noise_mean;
    ev.noise_deviation = noise_deviation;
    ev.dwell_samples = dwell;
    ev.quality_flags = 0U;
    ev.valid = 1U;

    if (dwell >= MIN_SHAPE_DWELL) {
        ev.quality_flags |= 0x01U;
    }
    if (peak_sta_lta_q8 >= STA_LTA_RATIO_Q8) {
        ev.quality_flags |= 0x02U;
    }
    if (peak_env > detector_threshold(TRIGGER_MULTIPLIER)) {
        ev.quality_flags |= 0x04U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    last_event = ev;
    last_event_sample_index = ev.sample_index;
    last_event_timestamp_us = ev.timestamp_us;
    if (primask == 0U) {
        __enable_irq();
    }

    /* User-requested radio mode: transmit ev.timestamp_us in the 64-bit packet field. */
    Radio_PublishEventTimestamp(ev.timestamp_us);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

static void detector_process_pcm16(int16_t pcm, uint64_t sample_index)
{
    uint32_t rect = u32_abs_i32((int32_t)pcm);
    int32_t env_delta = (int32_t)rect - (int32_t)env_state;
    int32_t new_env = (int32_t)env_state + (env_delta >> ENV_SHIFT);
    uint32_t env = (new_env < 0) ? 0U : (uint32_t)new_env;
    uint32_t threshold;

    env_state = env;
    pretrigger_pcm[pretrigger_wr] = pcm;
    pretrigger_env[pretrigger_wr] = (env > 65535U) ? 65535U : (uint16_t)env;
    pretrigger_wr = (pretrigger_wr + 1U) & PRETRIGGER_MASK;

    update_sta_lta(env);
    update_noise_if_quiet(env);
    update_cusum(env);
    threshold = detector_threshold(TRIGGER_MULTIPLIER);

    /* Обновление диагностических переменных для STM32CubeMonitor.
     * dbg_rect16      - abs(pcm16), выпрямленный 16-битный сигнал;
     * dbg_env         - сглаженная огибающая;
     * dbg_thr_trigger - основной адаптивный порог детектирования;
     * dbg_thr_onset   - нижний порог уточнения начала события.
     */
    dbg_rect16 = (rect > 65535U) ? 65535U : (uint16_t)rect;
    dbg_env = env;
    dbg_thr_trigger = threshold;
    dbg_thr_onset = detector_threshold(ONSET_MULTIPLIER);

    switch (det_state) {
    case DET_STATE_QUIET:
        if (env > threshold) {
            det_state = DET_STATE_CANDIDATE;
            candidate_age = 0U;
            candidate_peak = env;
            candidate_peak_sta_lta_q8 = sta_lta_ratio_q8();
            candidate_dwell = 1U;
            candidate_start_sample = sample_index;
        }
        break;

    case DET_STATE_CANDIDATE:
        candidate_age++;
        if (env > candidate_peak) {
            candidate_peak = env;
        }
        {
            uint32_t ratio = sta_lta_ratio_q8();
            if (ratio > candidate_peak_sta_lta_q8) {
                candidate_peak_sta_lta_q8 = ratio;
            }
        }
        if (env > threshold && candidate_dwell < 65535U) {
            candidate_dwell++;
        }

        if (sta_lta_confirmed() && cusum_pos >= CUSUM_CONFIRM) {
            uint64_t ts_sample = refine_first_arrival(sample_index);
            if (ts_sample > candidate_start_sample) {
                ts_sample = candidate_start_sample;
            }
            publish_event(ts_sample, candidate_peak, candidate_peak_sta_lta_q8, candidate_dwell);
            det_state = DET_STATE_LOCKOUT;
            lockout_left = LOCKOUT_SAMPLES;
            cusum_pos = 0U;
        } else if (candidate_age > CANDIDATE_TIMEOUT && env < threshold) {
            det_state = DET_STATE_QUIET;
            cusum_pos = 0U;
        }
        break;

    case DET_STATE_LOCKOUT:
        if (lockout_left > 0U) {
            lockout_left--;
        } else {
            det_state = DET_STATE_QUIET;
            candidate_age = 0U;
            candidate_peak = 0U;
            candidate_peak_sta_lta_q8 = 0U;
            candidate_dwell = 0U;
        }
        break;

    default:
        det_state = DET_STATE_QUIET;
        break;
    }

    /* Состояние после выполнения переходов автомата за текущий отсчёт.
     * Расшифровка для графиков: 0 - Фон, 1 - Кандидат, 2 - Блокировка.
     */
    dbg_det_state = (uint8_t)det_state;
}

void process_audio_block(uint16_t *buf, uint16_t len, uint64_t block_start_sample)
{
    for (uint16_t i = 0; i < len; i++) {
        int32_t sample24 = decode_inmp441_sample24(buf, i);
        test24 = sample24;

        dc_estimate += (sample24 - dc_estimate) >> DC_SHIFT;
        int32_t filtered24 = sample24 - dc_estimate;
        int32_t pcm32 = filtered24 >> 8;

        if (pcm32 > 32767) {
            pcm32 = 32767;
        } else if (pcm32 < -32768) {
            pcm32 = -32768;
        }

        detector_process_pcm16((int16_t)pcm32, block_start_sample + i);
    }
}
static void nrf3_init_ptx_timestamp(void)
{
    nrf_ce_low(&nrf3_tx);
    HAL_Delay(5U);

    nrf_write_reg(&nrf3_tx, NRF_REG_CONFIG, 0x0CU);       /* PWR_DOWN, 2-byte CRC, PTX */
    nrf_clear_all_irqs_and_fifos(&nrf3_tx, 1U);
    nrf_write_reg(&nrf3_tx, NRF_REG_EN_AA, 0x01U);        /* Auto ACK on pipe 0 */
    nrf_write_reg(&nrf3_tx, NRF_REG_EN_RXADDR, 0x01U);    /* Pipe 0 enabled for ACK reception */
    nrf_write_reg(&nrf3_tx, NRF_REG_SETUP_AW, 0x03U);     /* 5-byte addresses */
    nrf_write_reg(&nrf3_tx, NRF_REG_SETUP_RETR, 0x3FU);   /* 750 us, 15 retries */
    nrf_write_reg(&nrf3_tx, NRF_REG_RF_CH, NRF3_FREQ);
    nrf_write_reg(&nrf3_tx, NRF_REG_RF_SETUP, 0x0EU);     /* 2 Mbps, 0 dBm */
    nrf_write_multi(&nrf3_tx, NRF_REG_TX_ADDR, nrf3_master_pipe0_addr, RADIO_ADDR_WIDTH);
    nrf_write_multi(&nrf3_tx, NRF_REG_RX_ADDR_P0, nrf3_master_pipe0_addr, RADIO_ADDR_WIDTH);
    nrf_write_reg(&nrf3_tx, NRF_REG_DYNPD, 0x00U);
    nrf_write_reg(&nrf3_tx, NRF_REG_FEATURE, 0x00U);
    nrf_clear_all_irqs_and_fifos(&nrf3_tx, 1U);

    /* IRQ: RX_DR masked, TX_DS and MAX_RT unmasked. */
    nrf_write_reg(&nrf3_tx, NRF_REG_CONFIG, 0x4EU);
    HAL_Delay(2U);
    nrf_clear_all_irqs_and_fifos(&nrf3_tx, 1U);
}

static void nrf2_init_prx_control(void)
{
    nrf_ce_low(&nrf2_rx);
    HAL_Delay(5U);

    nrf_write_reg(&nrf2_rx, NRF_REG_CONFIG, 0x0CU);       /* PWR_DOWN, 2-byte CRC */
    nrf_clear_all_irqs_and_fifos(&nrf2_rx, 0U);
    nrf_write_reg(&nrf2_rx, NRF_REG_EN_AA, 0x03U);        /* Enhanced ShockBurst on pipe0+pipe1; NO_ACK bit suppresses ACK for sync/reset */
    nrf_write_reg(&nrf2_rx, NRF_REG_EN_RXADDR, 0x03U);    /* Pipe 0 common + pipe 1 individual */
    nrf_write_reg(&nrf2_rx, NRF_REG_SETUP_AW, 0x03U);     /* 5-byte addresses */
    nrf_write_reg(&nrf2_rx, NRF_REG_SETUP_RETR, 0x00U);
    nrf_write_reg(&nrf2_rx, NRF_REG_RF_CH, NRF2_FREQ);
    nrf_write_reg(&nrf2_rx, NRF_REG_RF_SETUP, 0x0EU);     /* 2 Mbps, 0 dBm */
    nrf_write_multi(&nrf2_rx, NRF_REG_RX_ADDR_P0, nrf2_common_pipe0_addr, RADIO_ADDR_WIDTH);
    nrf_write_multi(&nrf2_rx, NRF_REG_RX_ADDR_P1, nrf2_slave_pipe1_addr, RADIO_ADDR_WIDTH);
    nrf_write_reg(&nrf2_rx, NRF_REG_RX_PW_P0, RADIO_CTRL_PAYLOAD_LEN);
    nrf_write_reg(&nrf2_rx, NRF_REG_RX_PW_P1, RADIO_CTRL_PAYLOAD_LEN);
    nrf_write_reg(&nrf2_rx, NRF_REG_DYNPD, 0x00U);
    nrf_write_feature_checked(&nrf2_rx, 0x01U);           /* Accept NO_ACK packets from master */
    nrf_clear_all_irqs_and_fifos(&nrf2_rx, 0U);

    /* IRQ: RX_DR unmasked, TX_DS/MAX_RT masked, PRX enabled. */
    nrf_write_reg(&nrf2_rx, NRF_REG_CONFIG, 0x3FU);
    HAL_Delay(2U);
    nrf_clear_all_irqs_and_fifos(&nrf2_rx, 0U);
    nrf_ce_high(&nrf2_rx);
    HAL_Delay(2U);
}

static void Radio_Init(void)
{
    nrf_csn_high(&nrf2_rx);
    nrf_csn_high(&nrf3_tx);
    nrf_ce_low(&nrf2_rx);
    nrf_ce_low(&nrf3_tx);
    radio_nrf2_irq_pending = 0U;
    radio_nrf3_irq_pending = 0U;
    radio_nrf3_tx_busy = 0U;
    nrf3_init_ptx_timestamp();
    nrf2_init_prx_control();
    Radio_UpdateNrfDebugSnapshot();
}

static void nrf_read_payload_buf(const nrf_dev_t *dev, uint8_t *payload, uint8_t len)
{
    nrf_csn_low(dev);
    nrf_spi_xfer(dev, NRF_CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0U; i < len; i++) {
        payload[i] = nrf_spi_xfer(dev, 0x00U);
    }
    nrf_csn_high(dev);
}

static uint64_t get_u64_le(const uint8_t *p)
{
    return ((uint64_t)p[0]) |
           ((uint64_t)p[1] << 8) |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

static void nrf3_send_event_packet(const radio_event_packet_t *pkt)
{
    nrf_ce_low(&nrf3_tx);
    nrf_write_reg(&nrf3_tx, NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    nrf_command(&nrf3_tx, NRF_CMD_FLUSH_TX);

    nrf_csn_low(&nrf3_tx);
    nrf_spi_xfer(&nrf3_tx, NRF_CMD_W_TX_PAYLOAD);
    const uint8_t *p = (const uint8_t *)pkt;
    for (uint8_t i = 0; i < RADIO_EVENT_PAYLOAD_LEN; i++) {
        nrf_spi_xfer(&nrf3_tx, p[i]);
    }
    nrf_csn_high(&nrf3_tx);

    radio_nrf3_tx_busy = 1U;
    nrf_ce_high(&nrf3_tx);
    delay_us_blocking(20U);
    nrf_ce_low(&nrf3_tx);
}

static uint64_t Radio_ApplySyncCorrection(uint64_t timestamp_us)
{
    if (SynEnable == 0U) {
        return timestamp_us;
    }

    int64_t corrected = (int64_t)timestamp_us + (int64_t)radio_time_correction_us;
    if (corrected < 0) {
        corrected = 0;
    }
    return (uint64_t)corrected;
}

void Radio_ProcessMainLoop(void)
{
    static uint32_t last_debug_snapshot_ms = 0U;
    uint8_t handle_nrf2 = 0U;
    uint8_t handle_nrf3 = 0U;

    /* EXTI only posts flags; all nRF SPI transactions are executed here,
     * outside the interrupt context, so I2S DMA callbacks are not blocked. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (radio_nrf2_irq_pending != 0U) {
        radio_nrf2_irq_pending = 0U;
        handle_nrf2 = 1U;
    }
    if (radio_nrf3_irq_pending != 0U) {
        radio_nrf3_irq_pending = 0U;
        handle_nrf3 = 1U;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (handle_nrf2 != 0U) {
        Radio_HandleNrf2Rx();
    }
    if (handle_nrf3 != 0U) {
        Radio_HandleNrf3TxIrq();
    }

    if ((HAL_GetTick() - last_debug_snapshot_ms) >= 100U) {
        last_debug_snapshot_ms = HAL_GetTick();
        Radio_UpdateNrfDebugSnapshot();
    }

    if ((radio_event_ready != 0U) && (radio_nrf3_tx_busy == 0U)) {
        radio_event_packet_t pkt;

        primask = __get_PRIMASK();
        __disable_irq();
        pkt = radio_event_packet;
        radio_event_ready = 0U;
        if (primask == 0U) {
            __enable_irq();
        }

        pkt.timestamp_sample = Radio_ApplySyncCorrection(pkt.timestamp_sample);
        nrf3_send_event_packet(&pkt);
    }
}

void Radio_PublishEventTimestamp(uint64_t timestamp_us)
{
    radio_event_packet_t pkt;

    recognized_event_id++; /* uint8_t automatically wraps 255 -> 0. */
    pkt.timestamp_sample = timestamp_us; /* User-requested mode: transmit ev.timestamp_us in this field. */
    pkt.event_id = recognized_event_id;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    radio_event_packet = pkt;
    radio_event_ready = 1U;
    last_event_timestamp_us = timestamp_us;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void Radio_ResetInternalTimeAndEvents(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    recognized_event_id = 0U;
    radio_event_ready = 0U;
    radio_event_packet.timestamp_sample = 0U;
    radio_event_packet.event_id = 0U;
    last_event_timestamp_us = 0U;
    last_event_sample_index = 0U;
    last_event.valid = 0U;
    sample_counter = 0U;
    process_pending_mask = 0U;
    process_block_start_sample[0] = 0U;
    process_block_start_sample[1] = 0U;
    recording_start_us = 0U;
    dc_estimate = 0;
    env_state = 0U;
    noise_mean = 512U;
    noise_deviation = 64U;
    sta_sum = 0U;
    lta_sum = 1U;
    sta_pos = 0U;
    lta_pos = 0U;
    cusum_pos = 0U;
    det_state = DET_STATE_QUIET;
    candidate_age = 0U;
    candidate_peak = 0U;
    candidate_peak_sta_lta_q8 = 0U;
    candidate_dwell = 0U;
    pretrigger_wr = 0U;
    memset((void*)pretrigger_pcm, 0, sizeof(pretrigger_pcm));
    memset((void*)pretrigger_env, 0, sizeof(pretrigger_env));
    memset((void*)sta_ring, 0, sizeof(sta_ring));
    memset((void*)lta_ring, 0, sizeof(lta_ring));
    radio_time_correction_us = 0;
    radio_last_sync_error_us = 0;
    radio_sync_seen = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    set_time_us(0U);
    recording_start_us = get_time_us();
    nrf_command(&nrf2_rx, NRF_CMD_FLUSH_RX);
    nrf_command(&nrf3_tx, NRF_CMD_FLUSH_TX);
}
static void Radio_HandleSyncPacket(uint64_t master_time_us)
{
    uint64_t actual = get_time_us();
    radio_nrf2_sync_count++;
    radio_sync_seen = 1U;
    radio_nrf2_last_sync_master_us = master_time_us;
    radio_nrf2_last_sync_master_us_lo = (uint32_t)(master_time_us & 0xFFFFFFFFULL);
    radio_nrf2_last_sync_master_us_hi = (uint32_t)(master_time_us >> 32);

    int64_t diff64 = (int64_t)actual - (int64_t)master_time_us;
    if (diff64 > INT32_MAX) {
        diff64 = INT32_MAX;
    } else if (diff64 < INT32_MIN) {
        diff64 = INT32_MIN;
    }
    radio_last_sync_error_us = (int32_t)diff64;

    if (SynEnable != 0U) {
        set_time_us(master_time_us);
        //recording_start_us = get_time_us();
        radio_time_correction_us -= (int32_t)diff64;
    }
}

static void Radio_HandleNrf2Rx(void)
{
    uint8_t status = nrf_read_reg(&nrf2_rx, NRF_REG_STATUS);
    radio_nrf2_last_status = status;

    if ((status & NRF_STATUS_RX_DR) != 0U) {
        uint8_t pipe = (uint8_t)((status & NRF_STATUS_RX_P_NO) >> 1);
        uint8_t rx_payload[RADIO_CTRL_PAYLOAD_LEN] = {0U};
        nrf_read_payload_buf(&nrf2_rx, rx_payload, RADIO_CTRL_PAYLOAD_LEN);

        radio_nrf2_last_pipe = pipe;
        radio_nrf2_last_payload = rx_payload[0];
        radio_nrf2_last_payload_len = RADIO_CTRL_PAYLOAD_LEN;

        if ((pipe == 0U) && (rx_payload[0] == RADIO_PKT_TIME_RESET)) {
            radio_nrf2_reset_count++;
            nrf_clear_irq(&nrf2_rx, NRF_STATUS_RX_DR);
            Radio_ResetInternalTimeAndEvents();
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,GPIO_PIN_SET);
            return;
        }
        if ((pipe == 0U) && (rx_payload[0] == RADIO_PKT_SYNC)) {
            uint64_t master_time_us = get_u64_le(&rx_payload[1]);
            Radio_HandleSyncPacket(master_time_us);
        } else if ((pipe == 1U) && (rx_payload[0] == RADIO_PKT_HEALTH)) {
            /* ACK is generated by nRF24L01+ hardware because EN_AA pipe1 is enabled. */
            radio_nrf2_health_count++;
        }
    }

    nrf_clear_irq(&nrf2_rx, status & (NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT));
}

static void Radio_HandleNrf3TxIrq(void)
{
    uint8_t status = nrf_read_reg(&nrf3_tx, NRF_REG_STATUS);
    radio_nrf3_last_status = status;

    if ((status & NRF_STATUS_MAX_RT) != 0U) {
        radio_nrf3_max_rt_count++;
        nrf_clear_irq(&nrf3_tx, NRF_STATUS_MAX_RT);
        nrf_command(&nrf3_tx, NRF_CMD_FLUSH_TX);
        radio_nrf3_tx_busy = 0U;
    }

    if ((status & NRF_STATUS_TX_DS) != 0U) {
        radio_nrf3_tx_ok_count++;
        nrf_clear_irq(&nrf3_tx, NRF_STATUS_TX_DS);
        radio_nrf3_tx_busy = 0U;
    }
}

void Radio_IRQ_Handler(uint16_t GPIO_Pin)
{
    /* Never use HAL_SPI_* inside EXTI. NRF IRQ can stay low while STATUS is
     * uncleared; servicing it here may block DMA/I2S callbacks for milliseconds. */
    if (GPIO_Pin == NRF2_IRQ_Pin) {
        radio_nrf2_irq_pending = 1U;
    } else if (GPIO_Pin == NRF3_IRQ_Pin) {
        radio_nrf3_irq_pending = 1U;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

   HAL_TIM_Base_Start_IT(&htim2);
  recording_start_us = get_time_us();

  Radio_Init();

  /*
   * Запуск DMA приёма. Для 24-битного I2S каждый сэмпл = 2 полуслова.
   * Размер = BLOCK_SAMPLES сэмплов * 2 половины * 2 полуслова = BLOCK_SAMPLES * 4
   */
  HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*)i2s_buffer, I2S_HAL_RX_SIZE_PARAM);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint16_t *block_ptr = NULL;
    uint64_t block_start_sample = 0U;

    while (pop_dma_half(&block_ptr, &block_start_sample) != 0xFFU) {
        process_audio_block(block_ptr, BLOCK_SAMPLES, block_start_sample);
        block_ptr = NULL;
    }

    /* Радио обслуживается только когда аудио-очередь пуста. Это защищает
     * непрерывный I2S/DMA тракт от блокировок SPI/nRF и STM32CubeMonitor. */
    if (!audio_has_pending_block()) {
        Radio_ProcessMainLoop();
    }
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;      /* Оставляем 24 бита */
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(NRF2_CSN_GPIO_Port, NRF2_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, NRF2_CE_Pin|NRF3_CE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF3_CSN_GPIO_Port, NRF3_CSN_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF2_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(NRF2_CSN_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF2_IRQ_Pin|NRF3_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  GPIO_InitStruct.Pin = NRF2_CE_Pin|NRF3_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF3_CSN_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
