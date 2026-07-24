#ifndef W5500_PORT_H
#define W5500_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * W5500 Lite network settings.
 * Change these values for your local network and server.
 */
#ifndef W5500_MAC0
#define W5500_MAC0              0x00U
#define W5500_MAC1              0x08U
#define W5500_MAC2              0xDCU
#define W5500_MAC3              0x12U
#define W5500_MAC4              0x34U
#define W5500_MAC5              0x56U
#endif

#ifndef W5500_IP0
#define W5500_IP0               192U
#define W5500_IP1               168U
#define W5500_IP2               1U
#define W5500_IP3               50U
#endif

#ifndef W5500_NETMASK0
#define W5500_NETMASK0          255U
#define W5500_NETMASK1          255U
#define W5500_NETMASK2          255U
#define W5500_NETMASK3          0U
#endif

#ifndef W5500_GATEWAY0
#define W5500_GATEWAY0          192U
#define W5500_GATEWAY1          168U
#define W5500_GATEWAY2          1U
#define W5500_GATEWAY3          1U
#endif

#ifndef W5500_DNS0
#define W5500_DNS0              8U
#define W5500_DNS1              8U
#define W5500_DNS2              8U
#define W5500_DNS3              8U
#endif

#ifndef TDOA_SERVER_IP0
#define TDOA_SERVER_IP0         192U
#define TDOA_SERVER_IP1         168U
#define TDOA_SERVER_IP2         1U
#define TDOA_SERVER_IP3         100U
#endif

#ifndef TDOA_SERVER_PORT
#define TDOA_SERVER_PORT        5039U
#endif

#define W5500_TEST_SOCKET       0U
#define W5500_LOCAL_TCP_PORT    50000U
#define W5500_VERSION_VALUE     0x04U
#define TDOA_SERVER_PAYLOAD_LEN 36U
#define TDOA_SERVER_FRAME_LEN   40U

#define TDOA_W5500_SOCKET        0U
#define TDOA_W5500_LOCAL_PORT    50000U

/* Public status codes returned by this port layer. */
typedef enum
{
    W5500_PORT_OK = 0,
    W5500_PORT_SPI_ERROR,
    W5500_PORT_VERSION_ERROR,
    W5500_PORT_PHY_LINK_DOWN,
    W5500_PORT_SOCKET_ERROR,
    W5500_PORT_CONNECT_TIMEOUT,
    W5500_PORT_SEND_ERROR
} w5500_port_status_t;

typedef struct
{
    uint8_t  node_id;
    uint64_t timestamp_us;
} tdoa_node_timestamp_t;

void W5500_Port_Init(void);
void W5500_Reset(void);
uint8_t W5500_ReadVersion(void);
w5500_port_status_t W5500_NetworkInit_Static(void);
w5500_port_status_t W5500_Init(void);
void W5500_Service(void);
void W5500_SetInterruptFlag(void);

void TDOA_BuildServerFrame(uint8_t out[TDOA_SERVER_FRAME_LEN],
                           const tdoa_node_timestamp_t nodes[4]);
void TDOA_BuildTestServerFrame(uint8_t out[TDOA_SERVER_FRAME_LEN]);
w5500_port_status_t TDOA_SendFrameToServer(const uint8_t frame[TDOA_SERVER_FRAME_LEN]);
w5500_port_status_t TDOA_SendTestPacketToServer(void);

#ifdef __cplusplus
}
#endif

#endif /* W5500_PORT_H */