#include "w5500_port.h"

#include "spi.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500.h"

#include <stddef.h>
#include <string.h>

extern SPI_HandleTypeDef hspi1;

static volatile uint8_t g_w5500_int_flag = 0U;

#define W5500_SPI_TIMEOUT_MS        100U
#define W5500_SPI_BURST_CHUNK       64U
#define W5500_RESET_READY_DELAY_MS  150U
#define W5500_VERSION_RETRY_COUNT   5U
#define W5500_VERSION_RETRY_DELAY_MS 5U

static void W5500_Select(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

static void W5500_Unselect(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

static uint8_t W5500_ReadByte(void)
{
    const uint8_t tx = 0xFFU;
    uint8_t rx = 0x00U;

    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1U, W5500_SPI_TIMEOUT_MS) != HAL_OK)
    {
        return 0x00U;
    }

    return rx;
}

static void W5500_WriteByte(uint8_t byte)
{
    uint8_t rx_dummy = 0U;
    (void)HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_dummy, 1U, W5500_SPI_TIMEOUT_MS);
}

static void W5500_ReadBurst(uint8_t *buffer, uint16_t len)
{
    uint8_t tx_dummy[W5500_SPI_BURST_CHUNK];

    if ((buffer == NULL) || (len == 0U))
    {
        return;
    }

    memset(tx_dummy, 0xFF, sizeof(tx_dummy));

    while (len > 0U)
    {
        uint16_t chunk = (len > W5500_SPI_BURST_CHUNK) ? W5500_SPI_BURST_CHUNK : len;

        if (HAL_SPI_TransmitReceive(&hspi1, tx_dummy, buffer, chunk, W5500_SPI_TIMEOUT_MS) != HAL_OK)
        {
            memset(buffer, 0, chunk);
            return;
        }

        buffer += chunk;
        len -= chunk;
    }
}

static void W5500_WriteBurst(uint8_t *buffer, uint16_t len)
{
    uint8_t rx_dummy[W5500_SPI_BURST_CHUNK];

    if ((buffer == NULL) || (len == 0U))
    {
        return;
    }

    while (len > 0U)
    {
        uint16_t chunk = (len > W5500_SPI_BURST_CHUNK) ? W5500_SPI_BURST_CHUNK : len;

        if (HAL_SPI_TransmitReceive(&hspi1, buffer, rx_dummy, chunk, W5500_SPI_TIMEOUT_MS) != HAL_OK)
        {
            return;
        }

        buffer += chunk;
        len -= chunk;
    }
}

static uint8_t W5500_ReadVersionRaw(void)
{
    uint8_t tx[4] = {0x00U, 0x39U, 0x00U, 0xFFU};
    uint8_t rx[4] = {0U, 0U, 0U, 0U};

    W5500_Select();
    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)sizeof(tx), W5500_SPI_TIMEOUT_MS) != HAL_OK)
    {
        W5500_Unselect();
        return 0U;
    }
    W5500_Unselect();

    return rx[3];
}

static void put_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v >> 0);
}

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

static uint8_t W5500_WaitSocketCommandDone(uint8_t sn, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (getSn_CR(sn) != 0U)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t W5500_SpiReadWriteSelfTest(void)
{
    uint8_t simr_original = getSIMR();
    uint8_t simr_test = (uint8_t)(simr_original ^ 0x01U);

    setSIMR(simr_test);
    if (getSIMR() != simr_test)
    {
        setSIMR(simr_original);
        return 0U;
    }

    setSIMR(simr_original);
    if (getSIMR() != simr_original)
    {
        return 0U;
    }

    return 1U;
}

static w5500_port_status_t W5500_OpenTcpClientSocket(uint8_t sn)
{
    close(sn);
    (void)W5500_WaitSocketCommandDone(sn, 100U);

    if (socket(sn, Sn_MR_TCP, W5500_LOCAL_TCP_PORT, 0U) != sn)
    {
        return W5500_PORT_SOCKET_ERROR;
    }

    uint32_t start = HAL_GetTick();
    while (getSn_SR(sn) != SOCK_INIT)
    {
        if ((HAL_GetTick() - start) >= 1000U)
        {
            close(sn);
            return W5500_PORT_SOCKET_ERROR;
        }
    }

    return W5500_PORT_OK;
}

void W5500_Reset(void)
{
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(W5500_RESET_READY_DELAY_MS);
}

uint8_t W5500_ReadVersion(void)
{
    uint8_t version = W5500_ReadVersionRaw();

    if (version == W5500_VERSION_VALUE)
    {
        return version;
    }

    version = getVERSIONR();
    if (version == W5500_VERSION_VALUE)
    {
        return version;
    }

    for (uint8_t i = 0U; i < W5500_VERSION_RETRY_COUNT; i++)
    {
        HAL_Delay(W5500_VERSION_RETRY_DELAY_MS);
        version = W5500_ReadVersionRaw();

        if (version == W5500_VERSION_VALUE)
        {
            break;
        }
    }

    return version;
}

void W5500_Port_Init(void)
{
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);

    W5500_Reset();

    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBurst, W5500_WriteBurst);
}

w5500_port_status_t W5500_NetworkInit_Static(void)
{
    uint8_t tx_size[8] = {2U, 2U, 2U, 2U, 2U, 2U, 2U, 2U};
    uint8_t rx_size[8] = {2U, 2U, 2U, 2U, 2U, 2U, 2U, 2U};

    wiz_NetInfo net_info = {
        .mac = {W5500_MAC0, W5500_MAC1, W5500_MAC2, W5500_MAC3, W5500_MAC4, W5500_MAC5},
        .ip  = {W5500_IP0, W5500_IP1, W5500_IP2, W5500_IP3},
        .sn  = {W5500_NETMASK0, W5500_NETMASK1, W5500_NETMASK2, W5500_NETMASK3},
        .gw  = {W5500_GATEWAY0, W5500_GATEWAY1, W5500_GATEWAY2, W5500_GATEWAY3},
        .dns = {W5500_DNS0, W5500_DNS1, W5500_DNS2, W5500_DNS3},
        .dhcp = NETINFO_STATIC
    };

    if (wizchip_init(tx_size, rx_size) != 0)
    {
        return W5500_PORT_SPI_ERROR;
    }

    wizchip_setnetinfo(&net_info);

    /* Enable socket 0 interrupts on W5500 INTn pin. */
    setSIMR(0x01U);
    setSn_IMR(TDOA_W5500_SOCKET,
              (uint8_t)(Sn_IR_CON |
                        Sn_IR_DISCON |
                        Sn_IR_RECV |
                        Sn_IR_TIMEOUT |
                        Sn_IR_SENDOK));

    return W5500_PORT_OK;
}

w5500_port_status_t W5500_Init(void)
{
    W5500_Port_Init();

    if (W5500_ReadVersion() != W5500_VERSION_VALUE)
    {
        return W5500_PORT_VERSION_ERROR;
    }

    if (W5500_SpiReadWriteSelfTest() == 0U)
    {
        return W5500_PORT_SPI_ERROR;
    }

    return W5500_NetworkInit_Static();
}

void W5500_SetInterruptFlag(void)
{
    g_w5500_int_flag = 1U;
}

void W5500_Service(void)
{
    if (g_w5500_int_flag == 0U)
    {
        return;
    }

    g_w5500_int_flag = 0U;

    uint8_t sir = getSIR();

    if ((sir & 0x01U) != 0U)
    {
        uint8_t sn_ir = getSn_IR(W5500_TEST_SOCKET);

        /* Clear W5500 socket interrupt bits by writing 1 to the asserted bits. */
        if (sn_ir != 0U)
        {
            setSn_IR(W5500_TEST_SOCKET, sn_ir);
        }
    }
}

void TDOA_BuildServerFrame(uint8_t out[TDOA_SERVER_FRAME_LEN],
                           const tdoa_node_timestamp_t nodes[4])
{
    if ((out == NULL) || (nodes == NULL))
    {
        return;
    }

    memset(out, 0, TDOA_SERVER_FRAME_LEN);

    /* Java server reads first 4 bytes as ByteBuffer.getInt(), therefore Big-Endian. */
    put_u32_be(&out[0], TDOA_SERVER_PAYLOAD_LEN);

    /* Payload format: 4 x (node_id:uint8 + timestamp_us:uint64 little-endian). */
    out[4] = nodes[0].node_id;
    put_u64_le(&out[5], nodes[0].timestamp_us);

    out[13] = nodes[1].node_id;
    put_u64_le(&out[14], nodes[1].timestamp_us);

    out[22] = nodes[2].node_id;
    put_u64_le(&out[23], nodes[2].timestamp_us);

    out[31] = nodes[3].node_id;
    put_u64_le(&out[32], nodes[3].timestamp_us);
}

void TDOA_BuildTestServerFrame(uint8_t out[TDOA_SERVER_FRAME_LEN])
{
    const tdoa_node_timestamp_t test_nodes[4] = {
        {.node_id = 1U, .timestamp_us = 206225ULL},
        {.node_id = 2U, .timestamp_us = 196802ULL},
        {.node_id = 3U, .timestamp_us = 204544ULL},
        {.node_id = 4U, .timestamp_us = 224370ULL}
    };

    TDOA_BuildServerFrame(out, test_nodes);
}

w5500_port_status_t TDOA_SendFrameToServer(const uint8_t frame[TDOA_SERVER_FRAME_LEN])
{
    uint8_t server_ip[4] = {TDOA_SERVER_IP0, TDOA_SERVER_IP1, TDOA_SERVER_IP2, TDOA_SERVER_IP3};
    uint8_t sn = W5500_TEST_SOCKET;
    int32_t sent;

    if (frame == NULL)
    {
        return W5500_PORT_SEND_ERROR;
    }

    w5500_port_status_t status = W5500_OpenTcpClientSocket(sn);
    if (status != W5500_PORT_OK)
    {
        return status;
    }

    if (connect(sn, server_ip, TDOA_SERVER_PORT) != SOCK_OK)
    {
        close(sn);
        return W5500_PORT_SOCKET_ERROR;
    }

    uint32_t start = HAL_GetTick();
    while (getSn_SR(sn) != SOCK_ESTABLISHED)
    {
        uint8_t sn_ir = getSn_IR(sn);
        if ((sn_ir & Sn_IR_TIMEOUT) != 0U)
        {
            setSn_IR(sn, Sn_IR_TIMEOUT);
            close(sn);
            return W5500_PORT_CONNECT_TIMEOUT;
        }

        if ((HAL_GetTick() - start) >= 3000U)
        {
            close(sn);
            return W5500_PORT_CONNECT_TIMEOUT;
        }
    }

    sent = send(sn, (uint8_t *)frame, TDOA_SERVER_FRAME_LEN);
    if (sent != (int32_t)TDOA_SERVER_FRAME_LEN)
    {
        disconnect(sn);
        close(sn);
        return W5500_PORT_SEND_ERROR;
    }

    start = HAL_GetTick();
    while ((getSn_IR(sn) & Sn_IR_SENDOK) == 0U)
    {
        uint8_t sn_ir = getSn_IR(sn);
        if ((sn_ir & Sn_IR_TIMEOUT) != 0U)
        {
            setSn_IR(sn, Sn_IR_TIMEOUT);
            disconnect(sn);
            close(sn);
            return W5500_PORT_SEND_ERROR;
        }

        if ((HAL_GetTick() - start) >= 1000U)
        {
            disconnect(sn);
            close(sn);
            return W5500_PORT_SEND_ERROR;
        }
    }
    setSn_IR(sn, Sn_IR_SENDOK);

    disconnect(sn);
    start = HAL_GetTick();
    while (getSn_SR(sn) != SOCK_CLOSED)
    {
        if ((HAL_GetTick() - start) >= 1000U)
        {
            close(sn);
            break;
        }
    }

    return W5500_PORT_OK;
}

w5500_port_status_t TDOA_SendTestPacketToServer(void)
{
    uint8_t frame[TDOA_SERVER_FRAME_LEN];

    TDOA_BuildTestServerFrame(frame);
    return TDOA_SendFrameToServer(frame);
}