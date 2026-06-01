#ifndef DWM3000_H
#define DWM3000_H

#include <stdint.h>
#include <esp_err.h>

// DW3000 / DWM3000 Register Addresses
#define DW3000_DEV_ID        0x00
#define DW3000_EUI          0x01
#define DW3000_PANADR       0x03
#define DW3000_SYS_CFG      0x04
#define DW3000_TX_FCTRL     0x08
#define DW3000_TX_BUFFER    0x09
#define DW3000_DX_TIME      0x0A
#define DW3000_RX_FWTO      0x0C
#define DW3000_SYS_CTRL     0x0D
#define DW3000_SYS_MASK     0x0E
#define DW3000_SYS_STATUS   0x0F
#define DW3000_RX_FINFO     0x10
#define DW3000_RX_BUFFER    0x11
#define DW3000_RX_FQUAL     0x12
#define DW3000_RX_TTCKI     0x13
#define DW3000_RX_TTCKO     0x14
#define DW3000_RX_TIME      0x15
#define DW3000_TX_TIME      0x17
#define DW3000_TX_ANTD      0x18
#define DW3000_SYS_STATE    0x19
#define DW3000_ACK_RESP_T   0x1A
#define DW3000_RX_SNIFF     0x1D
#define DW3000_TX_POWER     0x1E
#define DW3000_CHAN_CTRL    0x1F
#define DW3000_USR_SFD      0x21
#define DW3000_AGC_CTRL     0x23
#define DW3000_EXT_SYNC     0x24
#define DW3000_ACC_MEM      0x25
#define DW3000_GPIO_CTRL    0x26
#define DW3000_DRX_CONF     0x27
#define DW3000_RF_CONF      0x28
#define DW3000_TX_CAL       0x2A
#define DW3000_FS_CTRL      0x2B
#define DW3000_AON          0x2C
#define DW3000_OTP_IF       0x2D
#define DW3000_LDE_CTRL     0x2E
#define DW3000_DIG_DIAG     0x2F
#define DW3000_PMSC         0x36

// DW3000 Commands
#define DW3000_CMD_TXRXOFF  0x00
#define DW3000_CMD_TX       0x01
#define DW3000_CMD_RX       0x02

// Frame types for UWB communication
#define FRAME_TYPE_DATA     0x01
#define FRAME_TYPE_ACK      0x02
#define FRAME_TYPE_RANGE_REQ 0x03
#define FRAME_TYPE_RANGE_RESP 0x04

// UWB Communication Structure
typedef struct {
    uint8_t frame_type;
    uint8_t seq_num;
    uint8_t src_addr[2];
    uint8_t dst_addr[2];
    uint8_t payload[64];
    uint8_t payload_len;
} uwb_frame_t;

// Function declarations
esp_err_t dwm3000_init(void);
esp_err_t dwm3000_reset(void);
esp_err_t dwm3000_wakeup(void);
esp_err_t dwm3000_read_reg(uint16_t reg_addr, uint8_t *data, size_t len);
esp_err_t dwm3000_write_reg(uint16_t reg_addr, const uint8_t *data, size_t len);
esp_err_t dwm3000_send_frame(const uwb_frame_t *frame);
esp_err_t dwm3000_receive_frame(uwb_frame_t *frame, uint32_t timeout_ms);
esp_err_t dwm3000_start_rx(void);
esp_err_t dwm3000_stop_rx(void);
bool dwm3000_is_frame_ready(void);
esp_err_t dwm3000_set_address(uint16_t short_addr);
esp_err_t dwm3000_configure_channel(uint8_t channel);

#endif // DWM3000_H
