#include "master_radio.h"
#include "spi.h"
#include "tim.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/* ========================= User-tunable radio parameters ========================= */
#ifndef NRF2_FREQ
#define NRF2_FREQ                         40U
#endif
#ifndef NRF3_FREQ
#define NRF3_FREQ                         76U
#endif

#define NRF_SPI_TIMEOUT_MS                1U
#define NRF_PAYLOAD_SIZE_MAX              32U
#define NRF_ADDR_LEN                      5U

#define MASTER_SYNC_START_DELAY_MS        1000U
#define MASTER_SYNC_PERIOD_MS             100U
#define MASTER_HEALTH_PERIOD_MS           60000U
#define MASTER_EVENT_GROUP_TIMEOUT_MS     1000U

#define MASTER_CMD_RESET_TIME             0xA0U
#define MASTER_CMD_SYNC_TIME              0xA1U
#define MASTER_CMD_HEALTH_CHECK           0xA2U

#define NRF2_RESET_SYNC_ADDR              {0xD2U, 0xC2U, 0xB2U, 0xA2U, 0xE2U}
#define NRF2_HEALTH_ADDR_BASE             {0x10U, 0xC3U, 0xB3U, 0xA3U, 0xE3U}
#define NRF3_TIMESTAMP_ADDR_BASE          {0x00U, 0xCCU, 0xBBU, 0xAAU, 0xE1U}

/* ============================== nRF24L01+ commands ============================== */
#define NRF_CMD_R_REGISTER                0x00U
#define NRF_CMD_W_REGISTER                0x20U
#define NRF_CMD_R_RX_PAYLOAD              0x61U
#define NRF_CMD_W_TX_PAYLOAD              0xA0U
#define NRF_CMD_W_TX_PAYLOAD_NO_ACK       0xB0U
#define NRF_CMD_ACTIVATE                  0x50U
#define NRF_CMD_FLUSH_TX                  0xE1U
#define NRF_CMD_FLUSH_RX                  0xE2U
#define NRF_CMD_REUSE_TX_PL               0xE3U
#define NRF_CMD_NOP                       0xFFU

/* ============================== nRF24L01+ registers ============================== */
#define NRF_REG_CONFIG                    0x00U
#define NRF_REG_EN_AA                     0x01U
#define NRF_REG_EN_RXADDR                 0x02U
#define NRF_REG_SETUP_AW                  0x03U
#define NRF_REG_SETUP_RETR                0x04U
#define NRF_REG_RF_CH                     0x05U
#define NRF_REG_RF_SETUP                  0x06U
#define NRF_REG_STATUS                    0x07U
#define NRF_REG_OBSERVE_TX                0x08U
#define NRF_REG_RX_ADDR_P0                0x0AU
#define NRF_REG_RX_ADDR_P1                0x0BU
#define NRF_REG_RX_ADDR_P2                0x0CU
#define NRF_REG_RX_ADDR_P3                0x0DU
#define NRF_REG_RX_ADDR_P4                0x0EU
#define NRF_REG_RX_ADDR_P5                0x0FU
#define NRF_REG_TX_ADDR                   0x10U
#define NRF_REG_RX_PW_P0                  0x11U
#define NRF_REG_RX_PW_P1                  0x12U
#define NRF_REG_RX_PW_P2                  0x13U
#define NRF_REG_RX_PW_P3                  0x14U
#define NRF_REG_RX_PW_P4                  0x15U
#define NRF_REG_RX_PW_P5                  0x16U
#define NRF_REG_FIFO_STATUS               0x17U
#define NRF_REG_DYNPD                     0x1CU
#define NRF_REG_FEATURE                   0x1DU

#define NRF_STATUS_RX_DR                  0x40U
#define NRF_STATUS_TX_DS                  0x20U
#define NRF_STATUS_MAX_RT                 0x10U
#define NRF_STATUS_RX_P_NO_MASK           0x0EU
#define NRF_STATUS_RX_P_NO_SHIFT          1U
#define NRF_STATUS_TX_FULL                0x01U

#define NRF_CONFIG_MASK_RX_DR             0x40U
#define NRF_CONFIG_MASK_TX_DS             0x20U
#define NRF_CONFIG_MASK_MAX_RT            0x10U
#define NRF_CONFIG_EN_CRC                 0x08U
#define NRF_CONFIG_CRCO                   0x04U
#define NRF_CONFIG_PWR_UP                 0x02U
#define NRF_CONFIG_PRIM_RX                0x01U

#define NRF_FIFO_RX_EMPTY                 0x01U
#define NRF_FIFO_TX_EMPTY                 0x10U

#define NRF_FEATURE_EN_DYN_ACK            0x01U

/* =============================== Diagnostics ===================================== */
volatile uint32_t g_radio_timestamp_rx_count = 0U;
volatile uint32_t g_radio_grouped_event_count = 0U;
volatile uint32_t g_radio_nrf2_irq_count = 0U;
volatile uint32_t g_radio_nrf3_irq_count = 0U;
volatile uint32_t g_radio_nrf2_reset_tx_count = 0U;
volatile uint32_t g_radio_nrf2_sync_tx_count = 0U;
volatile uint32_t g_radio_nrf2_health_tx_count = 0U;
volatile uint32_t g_radio_nrf2_tx_ds_count = 0U;
volatile uint32_t g_radio_nrf2_max_rt_count = 0U;
volatile uint8_t  g_radio_nrf2_last_tx_cmd = 0U;
volatile uint8_t  g_radio_nrf2_last_tx_no_ack = 0U;
volatile uint8_t  g_radio_nrf2_last_payload_spi_cmd = 0U;
volatile uint8_t  g_radio_nrf2_status_before_tx = 0U;
volatile uint8_t  g_radio_nrf2_status_after_ce = 0U;
volatile uint8_t  g_radio_nrf2_tx_addr0 = 0U;
volatile uint8_t  g_radio_nrf2_addr_mode = 0U; /* 0=unknown, 1=common sync/reset, 2=health */
volatile uint8_t  g_radio_nrf2_rx_p0_addr0 = 0U;
volatile uint8_t  g_radio_health_current_index = 0xFFU;
volatile uint8_t  g_radio_health_waiting = 0U;
volatile uint8_t  g_radio_health_cycle_active = 0U;
volatile uint8_t  g_radio_health_status_0 = 0U;
volatile uint8_t  g_radio_health_status_1 = 0U;
volatile uint8_t  g_radio_health_status_2 = 0U;
volatile uint8_t  g_radio_health_status_3 = 0U;
volatile uint32_t g_radio_nrf2_common_addr_restore_count = 0U;
volatile uint32_t g_radio_nrf2_sync_deferred_count = 0U;
volatile uint8_t  g_radio_last_event_id = 0U;
volatile uint8_t  g_radio_last_pipe = 0xFFU;
volatile uint8_t  g_radio_health_mask = 0U;
volatile uint8_t  g_radio_nrf2_present = 0U;
volatile uint8_t  g_radio_nrf3_present = 0U;
volatile uint8_t  g_radio_nrf2_last_status = 0xFFU;
volatile uint8_t  g_radio_nrf2_last_fifo = 0xFFU;
volatile uint8_t  g_radio_nrf2_config = 0xFFU;
volatile uint8_t  g_radio_nrf2_observe_tx = 0xFFU;
volatile uint8_t  g_radio_nrf2_rf_ch_read = 0xFFU;
volatile uint8_t  g_radio_nrf2_setup_aw_read = 0xFFU;
volatile uint8_t  g_radio_nrf2_status_nop = 0xFFU;
volatile uint8_t  g_radio_nrf2_present_code = 0U;
volatile uint8_t  g_radio_nrf2_irq_pin_level = 0xFFU;
volatile uint32_t g_radio_nrf2_spi_error_count = 0U;
volatile uint8_t  g_radio_nrf2_status_after_clear = 0xFFU;
volatile uint8_t  g_radio_nrf2_feature = 0xFFU;
volatile uint8_t  g_radio_nrf2_dynpd = 0xFFU;
volatile uint8_t  g_radio_nrf3_last_status = 0xFFU;
volatile uint8_t  g_radio_nrf3_last_fifo = 0xFFU;
volatile uint8_t  g_radio_nrf3_config = 0xFFU;
volatile uint8_t  g_radio_nrf3_observe_tx = 0xFFU;
volatile uint8_t  g_radio_nrf3_rf_ch_read = 0xFFU;
volatile uint8_t  g_radio_nrf3_setup_aw_read = 0xFFU;
volatile uint8_t  g_radio_nrf3_status_nop = 0xFFU;
volatile uint8_t  g_radio_nrf3_present_code = 0U;
volatile uint8_t  g_radio_nrf3_irq_pin_level = 0xFFU;
volatile uint32_t g_radio_nrf3_spi_error_count = 0U;
volatile uint8_t  g_radio_nrf3_status_after_clear = 0xFFU;
volatile uint8_t  g_radio_nrf3_feature = 0xFFU;
volatile uint8_t  g_radio_nrf3_dynpd = 0xFFU;
volatile uint8_t  g_radio_nrf3_rx_dr_seen = 0U;
volatile uint8_t  g_radio_nrf3_poll_status = 0xFFU;
volatile uint32_t g_radio_nrf3_poll_rx_dr_count = 0U;
volatile uint32_t g_radio_nrf3_bad_pipe_count = 0U;
volatile uint32_t g_radio_nrf3_flush_count = 0U;
volatile uint32_t g_master_tim2_overflows = 0U;

/* =============================== Internal state ================================== */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *csn_port;
    uint16_t csn_pin;
    GPIO_TypeDef *ce_port;
    uint16_t ce_pin;
} nrf_bus_t;

typedef struct
{
    uint8_t active;
    uint8_t event_id;
    uint8_t mask;
    uint32_t deadline_ms;
    uint64_t timestamp_us[MASTER_RADIO_NODE_COUNT];
} master_event_group_t;

static const nrf_bus_t nrf2 = {
    .hspi = &hspi2,
    .csn_port = NRF2_CSN_GPIO_Port,
    .csn_pin = NRF2_CSN_Pin,
    .ce_port = NRF2_CE_GPIO_Port,
    .ce_pin = NRF2_CE_Pin
};

static const nrf_bus_t nrf3 = {
    .hspi = &hspi3,
    .csn_port = NRF3_CSN_GPIO_Port,
    .csn_pin = NRF3_CSN_Pin,
    .ce_port = NRF3_CE_GPIO_Port,
    .ce_pin = NRF3_CE_Pin
};

static volatile uint8_t nrf2_irq_pending = 0U;
static volatile uint8_t nrf3_irq_pending = 0U;

static master_event_group_t event_groups[MASTER_RADIO_NODE_COUNT];
static uint8_t health_status[MASTER_RADIO_NODE_COUNT] = {0U, 0U, 0U, 0U};
static uint8_t health_index = 0U;
static uint8_t health_waiting = 0U;
static uint8_t health_cycle_active = 0U;
static uint32_t health_deadline_ms = 0U;
static uint32_t next_sync_ms = 0U;
static uint32_t next_health_ms = 0U;
static uint32_t sync_start_ms = 0U;
static uint32_t next_debug_snapshot_ms = 0U;

/* =============================== Byte helpers ==================================== */
static void put_u64_le(uint8_t *p, uint64_t v)
{
    p[0] = (uint8_t)(v >> 0);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    p[4] = (uint8_t)(v >> 32);
    p[5] = (uint8_t)(v >> 40);
    p[6] = (uint8_t)(v >> 48);
    p[7] = (uint8_t)(v >> 56);
}

static uint64_t get_u64_le(const uint8_t *p)
{
    return ((uint64_t)p[0] << 0) |
           ((uint64_t)p[1] << 8) |
           ((uint64_t)p[2] << 16) |
           ((uint64_t)p[3] << 24) |
           ((uint64_t)p[4] << 32) |
           ((uint64_t)p[5] << 40) |
           ((uint64_t)p[6] << 48) |
           ((uint64_t)p[7] << 56);
}

static uint64_t master_time_us(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t hi;
    uint32_t lo;

    __disable_irq();
    hi = g_master_tim2_overflows;
    lo = TIM2->CNT;
    if (((TIM2->SR & TIM_SR_UIF) != 0U) && (lo < 0x80000000UL))
    {
        hi++;
        lo = TIM2->CNT;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return (((uint64_t)hi) << 32) | (uint64_t)lo;
}

/* =============================== nRF low-level =================================== */
static void nrf_csn_low(const nrf_bus_t *dev)
{
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_RESET);
}

static void nrf_csn_high(const nrf_bus_t *dev)
{
    HAL_GPIO_WritePin(dev->csn_port, dev->csn_pin, GPIO_PIN_SET);
}

static void nrf_ce_low(const nrf_bus_t *dev)
{
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_RESET);
}

static void nrf_ce_high(const nrf_bus_t *dev)
{
    HAL_GPIO_WritePin(dev->ce_port, dev->ce_pin, GPIO_PIN_SET);
}

static uint8_t nrf_xfer(const nrf_bus_t *dev, uint8_t tx)
{
    uint8_t rx = 0xFFU;
    if (HAL_SPI_TransmitReceive(dev->hspi, &tx, &rx, 1U, NRF_SPI_TIMEOUT_MS) != HAL_OK)
    {
        if (dev == &nrf2)
        {
            g_radio_nrf2_spi_error_count++;
        }
        else if (dev == &nrf3)
        {
            g_radio_nrf3_spi_error_count++;
        }
        return 0xFFU;
    }
    return rx;
}

static uint8_t nrf_cmd(const nrf_bus_t *dev, uint8_t cmd)
{
    uint8_t status;
    nrf_csn_low(dev);
    status = nrf_xfer(dev, cmd);
    nrf_csn_high(dev);
    return status;
}

static void nrf_activate_features(const nrf_bus_t *dev)
{
    /* nRF24L01+ FEATURE/DYNPD registers may be locked after reset on many
     * modules. ACTIVATE 0x73 unlocks EN_DYN_ACK, which is required for
     * W_TX_PAYLOAD_NO_ACK reset/sync packets sent by NRF2. Call only once
     * during init: ACTIVATE toggles the feature-bank state on compatible chips. */
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, NRF_CMD_ACTIVATE);
    (void)nrf_xfer(dev, 0x73U);
    nrf_csn_high(dev);
}
static uint8_t nrf_read_reg(const nrf_bus_t *dev, uint8_t reg);
static void nrf_write_reg(const nrf_bus_t *dev, uint8_t reg, uint8_t value);
static void nrf_write_feature_checked(const nrf_bus_t *dev, uint8_t value)
{
    /* ACTIVATE 0x73 toggles FEATURE access. If MCU resets while nRF stays
     * powered, an unconditional ACTIVATE can disable FEATURE again. Therefore
     * first try writing FEATURE; ACTIVATE only if readback is not equal. */
    nrf_write_reg(dev, NRF_REG_FEATURE, value);
    if (nrf_read_reg(dev, NRF_REG_FEATURE) != value) {
        nrf_activate_features(dev);
        nrf_write_reg(dev, NRF_REG_FEATURE, value);
    }
}

static uint8_t nrf_get_status_nop(const nrf_bus_t *dev)
{
    uint8_t status;
    nrf_csn_low(dev);
    status = nrf_xfer(dev, NRF_CMD_NOP);
    nrf_csn_high(dev);
    return status;
}

static uint8_t nrf_read_reg(const nrf_bus_t *dev, uint8_t reg)
{
    uint8_t v;
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, (uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
    v = nrf_xfer(dev, NRF_CMD_NOP);
    nrf_csn_high(dev);
    return v;
}

static void nrf_write_reg(const nrf_bus_t *dev, uint8_t reg, uint8_t value)
{
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, (uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    (void)nrf_xfer(dev, value);
    nrf_csn_high(dev);
}

static void nrf_read_buf(const nrf_bus_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, (uint8_t)(NRF_CMD_R_REGISTER | (reg & 0x1FU)));
    for (uint8_t i = 0U; i < len; i++)
    {
        buf[i] = nrf_xfer(dev, NRF_CMD_NOP);
    }
    nrf_csn_high(dev);
}

static void nrf_write_buf(const nrf_bus_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, (uint8_t)(NRF_CMD_W_REGISTER | (reg & 0x1FU)));
    for (uint8_t i = 0U; i < len; i++)
    {
        (void)nrf_xfer(dev, buf[i]);
    }
    nrf_csn_high(dev);
}

static void nrf_write_payload(const nrf_bus_t *dev, const uint8_t *payload, uint8_t len, uint8_t no_ack)
{
    uint8_t cmd = no_ack ? NRF_CMD_W_TX_PAYLOAD_NO_ACK : NRF_CMD_W_TX_PAYLOAD;
    if (dev == &nrf2) {
        g_radio_nrf2_last_payload_spi_cmd = cmd;
    }
    nrf_csn_low(dev);
    (void)nrf_xfer(dev, cmd);
    for (uint8_t i = 0U; i < len; i++)
    {
        (void)nrf_xfer(dev, payload[i]);
    }
    nrf_csn_high(dev);
}

static uint8_t nrf_read_payload(const nrf_bus_t *dev, uint8_t *payload, uint8_t len)
{
    uint8_t status;
    nrf_csn_low(dev);
    status = nrf_xfer(dev, NRF_CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0U; i < len; i++)
    {
        payload[i] = nrf_xfer(dev, NRF_CMD_NOP);
    }
    nrf_csn_high(dev);
    return status;
}

static void nrf_clear_irqs(const nrf_bus_t *dev, uint8_t flags)
{
    nrf_write_reg(dev, NRF_REG_STATUS, (uint8_t)(flags & (NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)));
}

static void nrf_clear_all_irqs_and_fifos(const nrf_bus_t *dev)
{
    nrf_ce_low(dev);
    nrf_write_reg(dev, NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    (void)nrf_cmd(dev, NRF_CMD_FLUSH_RX);
    (void)nrf_cmd(dev, NRF_CMD_FLUSH_TX);
    nrf_write_reg(dev, NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    if (dev == &nrf2)
    {
        g_radio_nrf2_status_after_clear = nrf_read_reg(dev, NRF_REG_STATUS);
    }
    else if (dev == &nrf3)
    {
        g_radio_nrf3_status_after_clear = nrf_read_reg(dev, NRF_REG_STATUS);
    }
}

static void MasterRadio_UpdateNrfDebugSnapshot(void)
{
    g_radio_nrf2_status_nop = nrf_get_status_nop(&nrf2);
    g_radio_nrf3_status_nop = nrf_get_status_nop(&nrf3);

    g_radio_nrf2_last_status = nrf_read_reg(&nrf2, NRF_REG_STATUS);
    g_radio_nrf3_last_status = nrf_read_reg(&nrf3, NRF_REG_STATUS);

    g_radio_nrf2_last_fifo = nrf_read_reg(&nrf2, NRF_REG_FIFO_STATUS);
    g_radio_nrf3_last_fifo = nrf_read_reg(&nrf3, NRF_REG_FIFO_STATUS);

    g_radio_nrf2_config = nrf_read_reg(&nrf2, NRF_REG_CONFIG);
    g_radio_nrf3_config = nrf_read_reg(&nrf3, NRF_REG_CONFIG);

    g_radio_nrf2_observe_tx = nrf_read_reg(&nrf2, NRF_REG_OBSERVE_TX);
    g_radio_nrf3_observe_tx = nrf_read_reg(&nrf3, NRF_REG_OBSERVE_TX);

    g_radio_nrf2_rf_ch_read = nrf_read_reg(&nrf2, NRF_REG_RF_CH);
    g_radio_nrf3_rf_ch_read = nrf_read_reg(&nrf3, NRF_REG_RF_CH);

    g_radio_nrf2_setup_aw_read = nrf_read_reg(&nrf2, NRF_REG_SETUP_AW);
    g_radio_nrf3_setup_aw_read = nrf_read_reg(&nrf3, NRF_REG_SETUP_AW);
    g_radio_nrf2_feature = nrf_read_reg(&nrf2, NRF_REG_FEATURE);
    g_radio_nrf3_feature = nrf_read_reg(&nrf3, NRF_REG_FEATURE);
    g_radio_nrf2_dynpd = nrf_read_reg(&nrf2, NRF_REG_DYNPD);
    g_radio_nrf3_dynpd = nrf_read_reg(&nrf3, NRF_REG_DYNPD);

    g_radio_nrf2_irq_pin_level = (uint8_t)HAL_GPIO_ReadPin(NRF2_IRQ_GPIO_Port, NRF2_IRQ_Pin);
    g_radio_nrf3_irq_pin_level = (uint8_t)HAL_GPIO_ReadPin(NRF3_IRQ_GPIO_Port, NRF3_IRQ_Pin);

    g_radio_nrf2_present_code = 0U;
    if (g_radio_nrf2_rf_ch_read == NRF2_FREQ) { g_radio_nrf2_present_code |= 0x01U; }
    if (g_radio_nrf2_setup_aw_read == 0x03U) { g_radio_nrf2_present_code |= 0x02U; }
    if ((g_radio_nrf2_status_nop != 0xFFU) && (g_radio_nrf2_status_nop != 0x00U)) { g_radio_nrf2_present_code |= 0x04U; }
    g_radio_nrf2_present = ((g_radio_nrf2_present_code & 0x03U) == 0x03U) ? 1U : 0U;

    g_radio_nrf3_present_code = 0U;
    if (g_radio_nrf3_rf_ch_read == NRF3_FREQ) { g_radio_nrf3_present_code |= 0x01U; }
    if (g_radio_nrf3_setup_aw_read == 0x03U) { g_radio_nrf3_present_code |= 0x02U; }
    if ((g_radio_nrf3_status_nop != 0xFFU) && (g_radio_nrf3_status_nop != 0x00U)) { g_radio_nrf3_present_code |= 0x04U; }
    g_radio_nrf3_present = ((g_radio_nrf3_present_code & 0x03U) == 0x03U) ? 1U : 0U;
}

static void nrf2_reset_sync_addr(uint8_t out[NRF_ADDR_LEN]);
static void nrf2_health_addr(uint8_t slave_index, uint8_t out[NRF_ADDR_LEN]);

static void nrf_set_tx_addr(const nrf_bus_t *dev, const uint8_t addr[NRF_ADDR_LEN])
{
    nrf_write_buf(dev, NRF_REG_TX_ADDR, addr, NRF_ADDR_LEN);
    /* Required for Enhanced ShockBurst auto-ack on PTX. */
    nrf_write_buf(dev, NRF_REG_RX_ADDR_P0, addr, NRF_ADDR_LEN);
}

static void nrf2_apply_common_sync_addr(void)
{
    uint8_t addr[NRF_ADDR_LEN];

    nrf2_reset_sync_addr(addr);
    nrf_set_tx_addr(&nrf2, addr);
    g_radio_nrf2_tx_addr0 = addr[0];
    g_radio_nrf2_rx_p0_addr0 = addr[0];
    g_radio_nrf2_addr_mode = 1U;
    g_radio_nrf2_common_addr_restore_count++;
}

static void nrf2_apply_health_addr(uint8_t slave_index)
{
    uint8_t addr[NRF_ADDR_LEN];

    nrf2_health_addr(slave_index, addr);
    nrf_set_tx_addr(&nrf2, addr);
    g_radio_nrf2_tx_addr0 = addr[0];
    g_radio_nrf2_rx_p0_addr0 = addr[0];
    g_radio_nrf2_addr_mode = 2U;
    g_radio_health_current_index = slave_index;
}

static uint8_t nrf_check_present(const nrf_bus_t *dev)
{
    uint8_t original = nrf_read_reg(dev, NRF_REG_RF_CH);
    uint8_t test = (uint8_t)((original == 0x2AU) ? 0x15U : 0x2AU);
    nrf_write_reg(dev, NRF_REG_RF_CH, test);
    if (nrf_read_reg(dev, NRF_REG_RF_CH) != test)
    {
        return 0U;
    }
    nrf_write_reg(dev, NRF_REG_RF_CH, original);
    return 1U;
}

/* =============================== Address helpers ================================= */
static void nrf3_timestamp_addr(uint8_t slave_index, uint8_t out[NRF_ADDR_LEN])
{
    const uint8_t base[NRF_ADDR_LEN] = NRF3_TIMESTAMP_ADDR_BASE;
    memcpy(out, base, NRF_ADDR_LEN);
    out[0] = slave_index;
}

static void nrf2_reset_sync_addr(uint8_t out[NRF_ADDR_LEN])
{
    const uint8_t addr[NRF_ADDR_LEN] = NRF2_RESET_SYNC_ADDR;
    memcpy(out, addr, NRF_ADDR_LEN);
}

static void nrf2_health_addr(uint8_t slave_index, uint8_t out[NRF_ADDR_LEN])
{
    const uint8_t base[NRF_ADDR_LEN] = NRF2_HEALTH_ADDR_BASE;
    memcpy(out, base, NRF_ADDR_LEN);
    out[0] = (uint8_t)(0x10U + slave_index);
}

/* =============================== nRF configuration =============================== */
static void nrf3_init_timestamp_receiver(void)
{
    uint8_t addr[NRF_ADDR_LEN];

    nrf_ce_low(&nrf3);
    HAL_Delay(5U);

    nrf_clear_all_irqs_and_fifos(&nrf3);

    nrf_write_reg(&nrf3, NRF_REG_CONFIG, (uint8_t)(NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX));
    nrf_write_reg(&nrf3, NRF_REG_EN_AA, 0x0FU);       /* ACK for timestamp packets on pipes 0..3. */
    nrf_write_reg(&nrf3, NRF_REG_EN_RXADDR, 0x0FU);   /* Pipes 0..3. */
    nrf_write_reg(&nrf3, NRF_REG_SETUP_AW, 0x03U);    /* 5-byte addresses. */
    nrf_write_reg(&nrf3, NRF_REG_SETUP_RETR, 0x00U);  /* PRX: not used. */
    nrf_write_reg(&nrf3, NRF_REG_RF_CH, NRF3_FREQ);
    nrf_write_reg(&nrf3, NRF_REG_RF_SETUP, 0x0EU);    /* 2 Mbps, 0 dBm. */
    nrf_write_reg(&nrf3, NRF_REG_DYNPD, 0x00U);
    nrf_write_reg(&nrf3, NRF_REG_FEATURE, 0x00U);

    nrf3_timestamp_addr(0U, addr);
    nrf_write_buf(&nrf3, NRF_REG_RX_ADDR_P0, addr, NRF_ADDR_LEN);
    nrf3_timestamp_addr(1U, addr);
    nrf_write_buf(&nrf3, NRF_REG_RX_ADDR_P1, addr, NRF_ADDR_LEN);
    nrf3_timestamp_addr(2U, addr);
    nrf_write_reg(&nrf3, NRF_REG_RX_ADDR_P2, addr[0]);
    nrf3_timestamp_addr(3U, addr);
    nrf_write_reg(&nrf3, NRF_REG_RX_ADDR_P3, addr[0]);

    nrf_write_reg(&nrf3, NRF_REG_RX_PW_P0, MASTER_RADIO_EVENT_PAYLOAD_LEN);
    nrf_write_reg(&nrf3, NRF_REG_RX_PW_P1, MASTER_RADIO_EVENT_PAYLOAD_LEN);
    nrf_write_reg(&nrf3, NRF_REG_RX_PW_P2, MASTER_RADIO_EVENT_PAYLOAD_LEN);
    nrf_write_reg(&nrf3, NRF_REG_RX_PW_P3, MASTER_RADIO_EVENT_PAYLOAD_LEN);

    nrf_clear_all_irqs_and_fifos(&nrf3);
    HAL_Delay(2U);
    nrf_ce_high(&nrf3);
}

static void nrf2_init_sync_transmitter(void)
{
    uint8_t addr[NRF_ADDR_LEN];

    nrf_ce_low(&nrf2);
    HAL_Delay(5U);

    nrf_clear_all_irqs_and_fifos(&nrf2);

    nrf_write_reg(&nrf2, NRF_REG_CONFIG, (uint8_t)(NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO | NRF_CONFIG_PWR_UP));
    /* PTX receives ACK on pipe0. Health packets use ACK, sync/reset use W_TX_PAYLOAD_NO_ACK. */
    nrf_write_reg(&nrf2, NRF_REG_EN_AA, 0x01U);
    nrf_write_reg(&nrf2, NRF_REG_EN_RXADDR, 0x01U);   /* Pipe0 for ACK path when PTX. */
    nrf_write_reg(&nrf2, NRF_REG_SETUP_AW, 0x03U);    /* 5-byte addresses. */
    nrf_write_reg(&nrf2, NRF_REG_SETUP_RETR, 0xFFU);  /* 4000 us, 15 retries for health check. */
    nrf_write_reg(&nrf2, NRF_REG_RF_CH, NRF2_FREQ);
    nrf_write_reg(&nrf2, NRF_REG_RF_SETUP, 0x0EU);    /* 2 Mbps, 0 dBm. */
    nrf_write_feature_checked(&nrf2, NRF_FEATURE_EN_DYN_ACK);
    nrf_write_reg(&nrf2, NRF_REG_DYNPD, 0x00U);

    (void)addr;
    nrf2_apply_common_sync_addr();

    nrf_clear_all_irqs_and_fifos(&nrf2);
    HAL_Delay(2U);
}

/* =============================== TX operations =================================== */
static void nrf2_send_payload_start(const uint8_t *payload, uint8_t len, uint8_t no_ack)
{
    nrf_ce_low(&nrf2);
    g_radio_nrf2_status_before_tx = nrf_read_reg(&nrf2, NRF_REG_STATUS);
    (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
    nrf_clear_irqs(&nrf2, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    g_radio_nrf2_last_tx_no_ack = no_ack;
    if ((payload != NULL) && (len > 0U)) {
        g_radio_nrf2_last_tx_cmd = payload[0];
    }
    nrf_write_payload(&nrf2, payload, len, no_ack);
    nrf_ce_high(&nrf2);
    g_radio_nrf2_status_after_ce = nrf_read_reg(&nrf2, NRF_REG_STATUS);
}

static void nrf2_finish_tx_pulse(void)
{
    /* nRF24L01+ PTX requires CE high for at least 10 us. HAL_Delay(1) is
     * intentionally used here: sync packets are sparse and the master has no
     * real-time audio path, while the delay keeps the pulse independent of DWT. */
    HAL_Delay(1U);
    nrf_ce_low(&nrf2);
}

static void send_reset_time(void)
{
    uint8_t payload[MASTER_RADIO_RESET_PAYLOAD_LEN] = {0U};
    payload[0] = MASTER_CMD_RESET_TIME;

    nrf2_apply_common_sync_addr();
    nrf2_send_payload_start(payload, sizeof(payload), 1U);
    g_radio_nrf2_reset_tx_count++;
    nrf2_finish_tx_pulse();
}

static void send_sync_time(void)
{
    uint8_t payload[MASTER_RADIO_SYNC_PAYLOAD_LEN];

    payload[0] = MASTER_CMD_SYNC_TIME;
    put_u64_le(&payload[1], master_time_us());

    nrf2_apply_common_sync_addr();
    nrf2_send_payload_start(payload, sizeof(payload), 1U);
    g_radio_nrf2_sync_tx_count++;
    nrf2_finish_tx_pulse();
}

static void start_health_check_for_current_slave(void)
{
    uint8_t payload[MASTER_RADIO_HEALTH_PAYLOAD_LEN] = {0U};
    payload[0] = MASTER_CMD_HEALTH_CHECK;

    if (health_index >= MASTER_RADIO_NODE_COUNT)
    {
        health_waiting = 0U;
        g_radio_health_waiting = 0U;
        return;
    }

    nrf2_apply_health_addr(health_index);
    nrf2_send_payload_start(payload, sizeof(payload), 0U);
    g_radio_nrf2_health_tx_count++;
    nrf2_finish_tx_pulse();
    health_waiting = 1U;
    g_radio_health_waiting = 1U;
    health_deadline_ms = HAL_GetTick() + 90U;
}

/* =============================== Server sending ================================== */
static void send_health_status_to_server(void)
{
    uint8_t frame[TDOA_SERVER_FRAME_LEN];
    tdoa_node_timestamp_t nodes[MASTER_RADIO_NODE_COUNT];
    uint8_t mask = 0U;

    for (uint8_t i = 0U; i < MASTER_RADIO_NODE_COUNT; i++)
    {
        nodes[i].node_id = (uint8_t)(i + 1U);
        nodes[i].timestamp_us = health_status[i] ? UINT64_MAX : 0ULL;
        if (health_status[i])
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    g_radio_health_mask = mask;
    TDOA_BuildServerFrame(frame, nodes);
    (void)TDOA_SendFrameToServer(frame);
}

static void send_event_group_to_server(const master_event_group_t *group)
{
    uint8_t frame[TDOA_SERVER_FRAME_LEN];
    tdoa_node_timestamp_t nodes[MASTER_RADIO_NODE_COUNT];

    for (uint8_t i = 0U; i < MASTER_RADIO_NODE_COUNT; i++)
    {
        nodes[i].node_id = (uint8_t)(i + 1U);
        nodes[i].timestamp_us = ((group->mask & (1U << i)) != 0U) ? group->timestamp_us[i] : 0ULL;
    }
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    TDOA_BuildServerFrame(frame, nodes);
    if (TDOA_SendFrameToServer(frame) == W5500_PORT_OK)
    {
        g_radio_grouped_event_count++;
    }
}

/* =============================== Event grouping ================================== */
static master_event_group_t *find_group(uint8_t event_id)
{
    for (uint8_t i = 0U; i < MASTER_RADIO_NODE_COUNT; i++)
    {
        if (event_groups[i].active && (event_groups[i].event_id == event_id))
        {
            return &event_groups[i];
        }
    }
    return NULL;
}

static master_event_group_t *allocate_group(uint8_t event_id)
{
    uint32_t now = HAL_GetTick();
    master_event_group_t *oldest = &event_groups[0];

    for (uint8_t i = 0U; i < MASTER_RADIO_NODE_COUNT; i++)
    {
        if (!event_groups[i].active)
        {
            memset(&event_groups[i], 0, sizeof(event_groups[i]));
            event_groups[i].active = 1U;
            event_groups[i].event_id = event_id;
            event_groups[i].deadline_ms = now + MASTER_EVENT_GROUP_TIMEOUT_MS;
            return &event_groups[i];
        }
        if ((int32_t)(event_groups[i].deadline_ms - oldest->deadline_ms) < 0)
        {
            oldest = &event_groups[i];
        }
    }

    send_event_group_to_server(oldest);
    memset(oldest, 0, sizeof(*oldest));
    oldest->active = 1U;
    oldest->event_id = event_id;
    oldest->deadline_ms = now + MASTER_EVENT_GROUP_TIMEOUT_MS;
    return oldest;
}

static void accept_timestamp_packet(uint8_t pipe, const uint8_t payload[MASTER_RADIO_EVENT_PAYLOAD_LEN])
{
    uint8_t node_index;
    uint8_t event_id;
    uint64_t timestamp_us;
    master_event_group_t *group;

    if (pipe >= MASTER_RADIO_NODE_COUNT)
    {
        return;
    }

    node_index = pipe;
    timestamp_us = get_u64_le(&payload[0]);
    event_id = payload[8];

    group = find_group(event_id);
    if (group == NULL)
    {
        group = allocate_group(event_id);
    }

    group->timestamp_us[node_index] = timestamp_us;
    group->mask |= (uint8_t)(1U << node_index);
    group->deadline_ms = HAL_GetTick() + MASTER_EVENT_GROUP_TIMEOUT_MS;

    g_radio_timestamp_rx_count++;
    g_radio_last_event_id = event_id;
    g_radio_last_pipe = pipe;

    if (group->mask == 0x0FU)
    {
        send_event_group_to_server(group);
        memset(group, 0, sizeof(*group));
    }
}

static void service_event_timeouts(void)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0U; i < MASTER_RADIO_NODE_COUNT; i++)
    {
        if (event_groups[i].active && ((int32_t)(now - event_groups[i].deadline_ms) >= 0))
        {
            send_event_group_to_server(&event_groups[i]);
            memset(&event_groups[i], 0, sizeof(event_groups[i]));
        }
    }
}

/* =============================== IRQ service ===================================== */
static void service_nrf3_timestamp_irq(void)
{
    uint8_t status = nrf_read_reg(&nrf3, NRF_REG_STATUS);
    g_radio_nrf3_last_status = status;
    g_radio_nrf3_last_fifo = nrf_read_reg(&nrf3, NRF_REG_FIFO_STATUS);

    while ((status & NRF_STATUS_RX_DR) != 0U)
    {
        uint8_t pipe = (uint8_t)((status & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_SHIFT);
        uint8_t payload[MASTER_RADIO_EVENT_PAYLOAD_LEN];

        if (pipe <= 3U)
        {
            g_radio_nrf3_rx_dr_seen = 1U;
            (void)nrf_read_payload(&nrf3, payload, sizeof(payload));
            accept_timestamp_packet(pipe, payload);
        }
        else
        {
            g_radio_nrf3_bad_pipe_count++;
            g_radio_nrf3_flush_count++;
            (void)nrf_cmd(&nrf3, NRF_CMD_FLUSH_RX);
            break;
        }

        nrf_clear_irqs(&nrf3, NRF_STATUS_RX_DR);
        status = nrf_read_reg(&nrf3, NRF_REG_STATUS);
        g_radio_nrf3_last_status = status;
        g_radio_nrf3_last_fifo = nrf_read_reg(&nrf3, NRF_REG_FIFO_STATUS);
        if ((g_radio_nrf3_last_fifo & NRF_FIFO_RX_EMPTY) != 0U)
        {
            break;
        }
    }

    nrf_clear_irqs(&nrf3, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
}

static void service_nrf2_tx_irq(void)
{
    uint8_t status = nrf_read_reg(&nrf2, NRF_REG_STATUS);

    if (health_waiting != 0U)
    {
        if ((status & NRF_STATUS_TX_DS) != 0U)
        {
            g_radio_nrf2_tx_ds_count++;
            health_status[health_index] = 1U;
            if (health_index == 0U) { g_radio_health_status_0 = 1U; }
            else if (health_index == 1U) { g_radio_health_status_1 = 1U; }
            else if (health_index == 2U) { g_radio_health_status_2 = 1U; }
            else if (health_index == 3U) { g_radio_health_status_3 = 1U; }
            nrf_clear_irqs(&nrf2, NRF_STATUS_TX_DS);
            health_index++;
            health_waiting = 0U;
            g_radio_health_waiting = 0U;
        }
        else if ((status & NRF_STATUS_MAX_RT) != 0U)
        {
            g_radio_nrf2_max_rt_count++;
            health_status[health_index] = 0U;
            if (health_index == 0U) { g_radio_health_status_0 = 0U; }
            else if (health_index == 1U) { g_radio_health_status_1 = 0U; }
            else if (health_index == 2U) { g_radio_health_status_2 = 0U; }
            else if (health_index == 3U) { g_radio_health_status_3 = 0U; }
            nrf_clear_irqs(&nrf2, NRF_STATUS_MAX_RT);
            (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
            health_index++;
            health_waiting = 0U;
            g_radio_health_waiting = 0U;
        }
    }
    else
    {
        nrf_clear_irqs(&nrf2, (uint8_t)(status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT | NRF_STATUS_RX_DR)));
        if ((status & NRF_STATUS_TX_DS) != 0U)
        {
            g_radio_nrf2_tx_ds_count++;
        }
        if ((status & NRF_STATUS_MAX_RT) != 0U)
        {
            g_radio_nrf2_max_rt_count++;
            (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
        }
    }
}

static void health_mark_current(uint8_t ok)
{
    if (health_index < MASTER_RADIO_NODE_COUNT)
    {
        health_status[health_index] = ok;
        if (health_index == 0U) { g_radio_health_status_0 = ok; }
        else if (health_index == 1U) { g_radio_health_status_1 = ok; }
        else if (health_index == 2U) { g_radio_health_status_2 = ok; }
        else if (health_index == 3U) { g_radio_health_status_3 = ok; }
    }
}

static void service_health_fsm(void)
{
    uint32_t now = HAL_GetTick();

    if (health_cycle_active == 0U)
    {
        return;
    }

    if (health_waiting != 0U)
    {
        uint8_t status = nrf_read_reg(&nrf2, NRF_REG_STATUS);
        g_radio_nrf2_last_status = status;

        if ((status & NRF_STATUS_TX_DS) != 0U)
        {
            g_radio_nrf2_tx_ds_count++;
            health_mark_current(1U);
            nrf_clear_irqs(&nrf2, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
            (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
            health_index++;
            health_waiting = 0U;
            g_radio_health_waiting = 0U;
            return;
        }

        if ((status & NRF_STATUS_MAX_RT) != 0U)
        {
            g_radio_nrf2_max_rt_count++;
            health_mark_current(0U);
            nrf_clear_irqs(&nrf2, NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS);
            (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
            health_index++;
            health_waiting = 0U;
            g_radio_health_waiting = 0U;
            return;
        }

        if ((int32_t)(now - health_deadline_ms) >= 0)
        {
            health_mark_current(0U);
            nrf_clear_irqs(&nrf2, NRF_STATUS_MAX_RT | NRF_STATUS_TX_DS);
            (void)nrf_cmd(&nrf2, NRF_CMD_FLUSH_TX);
            health_index++;
            health_waiting = 0U;
            g_radio_health_waiting = 0U;
            return;
        }

        return;
    }

    if (health_index < MASTER_RADIO_NODE_COUNT)
    {
        start_health_check_for_current_slave();
        return;
    }

    send_health_status_to_server();
    nrf2_apply_common_sync_addr();
    health_index = MASTER_RADIO_NODE_COUNT;
    health_waiting = 0U;
    health_cycle_active = 0U;
    health_cycle_active = 0U;
    g_radio_health_current_index = 0xFFU;
    g_radio_health_waiting = 0U;
    g_radio_health_cycle_active = 0U;
    next_health_ms = HAL_GetTick() + MASTER_HEALTH_PERIOD_MS;
}

/* =============================== Public API ====================================== */
void MasterRadio_Init(void)
{
    HAL_GPIO_WritePin(NRF2_CSN_GPIO_Port, NRF2_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NRF3_CSN_GPIO_Port, NRF3_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NRF2_CE_GPIO_Port, NRF2_CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF3_CE_GPIO_Port, NRF3_CE_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);

    /* Basic SPI smoke-test before full configuration. The final presence flags
     * are updated below from the configured RF_CH and SETUP_AW registers. */
    (void)nrf_check_present(&nrf2);
    (void)nrf_check_present(&nrf3);

    nrf2_init_sync_transmitter();
    nrf3_init_timestamp_receiver();
    MasterRadio_UpdateNrfDebugSnapshot();

    memset(event_groups, 0, sizeof(event_groups));
    memset(health_status, 0, sizeof(health_status));
    g_radio_health_mask = 0U;
    g_radio_health_status_0 = 0U;
    g_radio_health_status_1 = 0U;
    g_radio_health_status_2 = 0U;
    g_radio_health_status_3 = 0U;
    g_radio_health_current_index = 0xFFU;
    g_radio_health_waiting = 0U;
    g_radio_health_cycle_active = 0U;

    send_reset_time();

    sync_start_ms = HAL_GetTick() + MASTER_SYNC_START_DELAY_MS;
    next_sync_ms = sync_start_ms;
    next_health_ms = HAL_GetTick() + MASTER_HEALTH_PERIOD_MS;
    next_debug_snapshot_ms = HAL_GetTick() + 250U;
    health_index = MASTER_RADIO_NODE_COUNT;
    health_waiting = 0U;
    health_cycle_active = 0U;
}

void MasterRadio_OnTimerPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        g_master_tim2_overflows++;
    }
}

void MasterRadio_OnExti(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == NRF2_IRQ_Pin)
    {
        nrf2_irq_pending = 1U;
        g_radio_nrf2_irq_count++;
    }
    else if (GPIO_Pin == NRF3_IRQ_Pin)
    {
        nrf3_irq_pending = 1U;
        g_radio_nrf3_irq_count++;
    }
}

void MasterRadio_Service(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t need_nrf3_service = 0U;

    if (nrf3_irq_pending != 0U)
    {
        nrf3_irq_pending = 0U;
        need_nrf3_service = 1U;
    }

    /* Fallback: if EXTI edge was missed or IRQ line was already low before
     * enabling NVIC, still service NRF3. This is safe on master because there
     * is no real-time audio DMA path here. */
    if (HAL_GPIO_ReadPin(NRF3_IRQ_GPIO_Port, NRF3_IRQ_Pin) == GPIO_PIN_RESET)
    {
        need_nrf3_service = 1U;
    }

    if (need_nrf3_service == 0U)
    {
        uint8_t st = nrf_read_reg(&nrf3, NRF_REG_STATUS);
        g_radio_nrf3_poll_status = st;
        if ((st & NRF_STATUS_RX_DR) != 0U)
        {
            g_radio_nrf3_poll_rx_dr_count++;
            need_nrf3_service = 1U;
        }
    }

    if (need_nrf3_service != 0U)
    {
        service_nrf3_timestamp_irq();
    }

    if (nrf2_irq_pending != 0U)
    {
        nrf2_irq_pending = 0U;
        if (health_cycle_active == 0U)
        {
            service_nrf2_tx_irq();
        }
    }

    service_event_timeouts();

    if ((int32_t)(now - next_debug_snapshot_ms) >= 0)
    {
        MasterRadio_UpdateNrfDebugSnapshot();
        next_debug_snapshot_ms = now + 250U;
    }

    if ((int32_t)(now - sync_start_ms) >= 0)
    {
        if ((int32_t)(now - next_sync_ms) >= 0)
        {
            if ((health_waiting == 0U) && (health_index >= MASTER_RADIO_NODE_COUNT))
            {
                send_sync_time();
                next_sync_ms = now + MASTER_SYNC_PERIOD_MS;
            }
            else
            {
                /* Do not rewrite TX_ADDR/RX_ADDR_P0 while a health-check ACK
                 * transaction is active. A deferred sync will be sent as soon
                 * as the health cycle finishes and common address is restored. */
                g_radio_nrf2_sync_deferred_count++;
                next_sync_ms = now + MASTER_SYNC_PERIOD_MS;
            }
        }
    }

    if (health_cycle_active != 0U)
    {
        service_health_fsm();
    }
    else if ((int32_t)(now - next_health_ms) >= 0)
    {
        health_cycle_active = 1U;
        g_radio_health_cycle_active = 1U;
        health_index = 0U;
        health_waiting = 0U;
        g_radio_health_waiting = 0U;
        memset(health_status, 0, sizeof(health_status));
        g_radio_health_status_0 = 0U;
        g_radio_health_status_1 = 0U;
        g_radio_health_status_2 = 0U;
        g_radio_health_status_3 = 0U;
        service_health_fsm();
    }
}
