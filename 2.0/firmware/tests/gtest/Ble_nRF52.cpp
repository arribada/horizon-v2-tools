// Ble_nRF52.cpp - Bluetooth syshal component unit tests
//
// Copyright (C) 2018 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
extern "C" {
#include <assert.h>
#include <stdint.h>
#include "unity.h"
#include "Mocksyshal_spi.h"
#include "syshal_ble.h"
#include "nRF52x_regs.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

#include <list>
#include <vector>

using std::list;
using std::vector;

static uint8_t tx_buf[64][1024];
static uint8_t rx_buf[64][1024];
static uint16_t tx_size[64];
static uint16_t rx_size[64];
static unsigned int rx_buf_wr = 0;
static unsigned int rx_buf_rd = 0;
static unsigned int tx_buf_wr = 0;
static unsigned int tx_buf_rd = 0;
static uint8_t event_id;
static uint32_t event_error;
static uint16_t event_length;

#define EXPECT_ARRAY_EQ(TARTYPE, reference, actual, element_count) \
    {\
    TARTYPE* reference_ = static_cast<TARTYPE *> (reference); \
    TARTYPE* actual_ = static_cast<TARTYPE *> (actual); \
    for(int cmp_i = 0; cmp_i < element_count; cmp_i++ ){\
      EXPECT_EQ(reference_[cmp_i], actual_[cmp_i]);\
    }\
    }

void syshal_ble_event_handler(syshal_ble_event_t *event)
{
    event_id = event->event_id;
    event_error = event->error;
    event_length = event->send_complete.length;
}

class BleTest : public ::testing::Test {

    virtual void SetUp() {
        Mocksyshal_spi_Init();
        syshal_spi_transfer_StubWithCallback(syshal_spi_transfer_Callback);
        event_id = -1;
        event_error = -1;
        event_length = -1;
    }

    virtual void TearDown() {
        Mocksyshal_spi_Verify();
        Mocksyshal_spi_Destroy();
    }

public:
    static int syshal_spi_transfer_Callback(uint32_t instance, uint8_t *wr_data, uint8_t *rd_data, uint16_t size,
        int cmock_num_calls)
    {

        printf("wr_data: %p rd_data: %p sz: %u\n", wr_data, rd_data, size);
        if (tx_buf_wr == tx_buf_rd)
        {
            printf("[%u] Unexpected transfer\n", tx_buf_rd);
            assert(0);
        }

        if (tx_size[tx_buf_rd & 63] != size)
        {
            printf("[%u] Unexpected size: actual=%u vs expected=%u\n", tx_buf_rd, size, tx_size[tx_buf_rd & 63]);
            assert(0);
        }

        if (memcmp(tx_buf[tx_buf_rd & 63], wr_data, tx_size[tx_buf_rd & 63]))
        {
            printf("[%u] TX buffer mismatch\n", tx_buf_rd);
            printf("Expected:\n");
            for (unsigned int i = 0; i < size; i++)
                printf("%02x\n", tx_buf[tx_buf_rd & 63][i]);
            printf("Actual:\n");
            for (unsigned int i = 0; i < size; i++)
                printf("%02x\n", wr_data[i]);
            assert(0);
        }

        memcpy(rd_data, rx_buf[rx_buf_rd & 63], rx_size[rx_buf_rd & 63]);

        tx_buf_rd++;
        rx_buf_rd++;

        return SYSHAL_SPI_NO_ERROR;
    }

    void SpiTransfer(uint8_t *write, uint8_t *read, uint16_t size) {
        memcpy(tx_buf[tx_buf_wr & 63], write, size);
        tx_size[tx_buf_wr++ & 63] = size;
        if (read)
            memcpy(rx_buf[rx_buf_wr & 63], read, size);
        rx_size[rx_buf_wr++ & 63] = size;
    }

    void Init()
    {
        syshal_spi_init_ExpectAndReturn(0, SYSHAL_SPI_NO_ERROR);
        ReadAppVersion(0);
        IntEnable(NRF52_INT_TX_DATA_SENT | NRF52_INT_RX_DATA_READY);
        EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_init(0));
    }

    void ReadAppVersion(uint16_t app_version)
    {
        uint8_t out[3] = {0}, in[3];
        memset(out, 0, sizeof(out));
        in[0] = out[0] = NRF52_REG_ADDR_APP_VERSION;
        in[1] = app_version;
        in[2] = app_version >> 8;
        SpiTransfer(out, in, 3);
    }

    void ReadSoftDevVersion(uint16_t soft_dev_version)
    {
        uint8_t out[3] = {0}, in[3];
        memset(out, 0, sizeof(out));
        in[0] = out[0] = NRF52_REG_ADDR_SOFT_DEV_VERSION;
        in[1] = soft_dev_version;
        in[2] = soft_dev_version >> 8;
        SpiTransfer(out, in, 3);
    }

    void IntEnable(uint8_t mask)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_INT_ENABLE | NRF52_SPI_WRITE_NOT_READ_ADDR,
            mask
        };
        SpiTransfer(buf, NULL, 2);
    }

    void Reset()
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_MODE | NRF52_SPI_WRITE_NOT_READ_ADDR,
            NRF52_MODE_RESET
        };
        SpiTransfer(buf, NULL, 2);
    }

    void SetMode(uint8_t mode)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_MODE | NRF52_SPI_WRITE_NOT_READ_ADDR,
            mode
        };
        SpiTransfer(buf, NULL, 2);
    }

    void GetMode(uint8_t mode)
    {
        uint8_t out[2], in[2];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_MODE;
        in[1] = mode;
        SpiTransfer(out, in, 2);
    }

    void SetOwnUUID(uint8_t uuid[16])
    {
        uint8_t out[17];
        out[0] = NRF52_REG_ADDR_OWN_UUID | NRF52_SPI_WRITE_NOT_READ_ADDR;
        memcpy(&out[1], uuid, 16);
        SpiTransfer(out, NULL, 17);
    }

    void SetTargetUUID(uint8_t uuid[16])
    {
        uint8_t out[17];
        out[0] = NRF52_REG_ADDR_TARGET_UUID | NRF52_SPI_WRITE_NOT_READ_ADDR;
        memcpy(&out[1], uuid, 16);
        SpiTransfer(out, NULL, 17);
    }

    void GetTargetUUID(uint8_t uuid[16])
    {
        uint8_t in[17], out[17];
        in[0] = out[0] = NRF52_REG_ADDR_TARGET_UUID;
        memset(&out[1], 0, 16);
        memcpy(&in[1], uuid, 16);
        SpiTransfer(out, in, 17);
    }

    void SetFwUpgradeSize(uint32_t size)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_FW_UPGRADE_SIZE | NRF52_SPI_WRITE_NOT_READ_ADDR,
            (uint8_t)size,
            (uint8_t)(size >> 8),
            (uint8_t)(size >> 16),
            (uint8_t)(size >> 24)
        };
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SetFwUpgradeType(uint8_t type)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_FW_UPGRADE_TYPE | NRF52_SPI_WRITE_NOT_READ_ADDR,
            type
        };
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SetFwUpgradeCrc(uint32_t crc)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_FW_UPGRADE_CRC | NRF52_SPI_WRITE_NOT_READ_ADDR,
            (uint8_t)crc,
            (uint8_t)(crc >> 8),
            (uint8_t)(crc >> 16),
            (uint8_t)(crc >> 24)
        };
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SetBeaconInterval(uint16_t interval_ms)
    {
        uint8_t buf[] = {
            NRF52_REG_ADDR_BEACON_INTERVAL | NRF52_SPI_WRITE_NOT_READ_ADDR,
            (uint8_t)interval_ms,
            (uint8_t)(interval_ms >> 8)
        };
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SetBeaconPayload(uint8_t beacon_payload[31])
    {
        uint8_t buf[32];
        buf[0] = NRF52_REG_ADDR_BEACON_PAYLOAD | NRF52_SPI_WRITE_NOT_READ_ADDR;
        memcpy(&buf[1], beacon_payload, 31);
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SetScanResponse(uint8_t scan_payload[31])
    {
        uint8_t buf[32];
        buf[0] = NRF52_REG_ADDR_SCAN_RESPONSE | NRF52_SPI_WRITE_NOT_READ_ADDR;
        memcpy(&buf[1], scan_payload, 31);
        SpiTransfer(buf, NULL, sizeof(buf));
    }

    void SendData(uint8_t *buffer, uint16_t size)
    {
        uint8_t buf[513];
        buf[0] = NRF52_REG_ADDR_TX_DATA_PORT | NRF52_SPI_WRITE_NOT_READ_ADDR;
        memcpy(&buf[1], buffer, size);
        SpiTransfer(buf, NULL, size + 1);
    }

    void ReadIntStatus(uint8_t int_status)
    {
        uint8_t out[2], in[2];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_INT_STATUS;
        in[1] = int_status;
        SpiTransfer(out, in, 2);
    }

    void ReadErrorCode(uint8_t error_code)
    {
        uint8_t out[2], in[2];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_ERROR_CODE;
        in[1] = error_code;
        SpiTransfer(out, in, 2);
    }

    void ReadTxDataLength(uint16_t length)
    {
        uint8_t out[3], in[3];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_TX_DATA_LENGTH;
        in[1] = length;
        in[2] = length >> 8;
        SpiTransfer(out, in, 3);
    }

    void ReadRxDataLength(uint16_t length)
    {
        uint8_t out[3], in[3];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_RX_DATA_LENGTH;
        in[1] = length;
        in[2] = length >> 8;
        SpiTransfer(out, in, 3);
    }

    void ReadRxDataPort(uint8_t *buffer, uint16_t length)
    {
        uint8_t out[513], in[513];
        memset(out, 0, sizeof(out));
        out[0] = in[0] = NRF52_REG_ADDR_RX_DATA_PORT;
        memcpy(&in[1], buffer, length);
        SpiTransfer(out, in, length + 1);
    }

    void ExpectEvent(uint8_t id, uint32_t error, uint16_t length)
    {
        EXPECT_EQ(id, event_id);
        EXPECT_EQ(error, event_error);
        if (id == SYSHAL_BLE_EVENT_SEND_COMPLETE ||
            id == SYSHAL_BLE_EVENT_RECEIVE_COMPLETE)
            EXPECT_EQ(length, event_length);
    }
};

TEST_F(BleTest, BleInitOk)
{
    Init();
}

TEST_F(BleTest, BleTermOk)
{
    Init();
    syshal_spi_term_IgnoreAndReturn(SYSHAL_SPI_NO_ERROR);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_term());
}

TEST_F(BleTest, BleReset)
{
    Init();
    Reset();
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_reset());
}

TEST_F(BleTest, BleSetMode)
{
    Init();
    SetMode(SYSHAL_BLE_MODE_GATT_SERVER);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_set_mode(SYSHAL_BLE_MODE_GATT_SERVER));
}

TEST_F(BleTest, BleGetMode)
{
    syshal_ble_mode_t mode;
    Init();
    GetMode(SYSHAL_BLE_MODE_GATT_SERVER);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_get_mode(&mode));
    EXPECT_EQ(SYSHAL_BLE_MODE_GATT_SERVER, mode);
}

TEST_F(BleTest, BleGetVersion)
{
    uint32_t version;
    Init();
    ReadAppVersion(0x1111);
    ReadSoftDevVersion(0x2222);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_get_version(&version));
    EXPECT_EQ((uint32_t)0x22221111, version);
}

TEST_F(BleTest, BleSetOwnUUID)
{
    uint8_t uuid[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    Init();
    SetOwnUUID(uuid);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_set_own_uuid(uuid));
}

TEST_F(BleTest, BleSetTargetUUID)
{
    uint8_t uuid[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    Init();
    SetTargetUUID(uuid);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_set_target_uuid(uuid));
}

TEST_F(BleTest, BleGetTargetUUID)
{
    uint8_t actual_uuid[16];
    uint8_t expected_uuid[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    Init();
    GetTargetUUID(expected_uuid);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_get_target_uuid(actual_uuid));
    EXPECT_ARRAY_EQ(uint8_t, expected_uuid, actual_uuid, 16);
}

TEST_F(BleTest, BleConfigFwUpgrade)
{
    Init();
    SetFwUpgradeSize(1024);
    SetFwUpgradeType(SYSHAL_BLE_FW_UPGRADE_TYPE_SOFT_DEV);
    SetFwUpgradeCrc(0x11223344);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_config_fw_upgrade(SYSHAL_BLE_FW_UPGRADE_TYPE_SOFT_DEV, 1024, 0x11223344));
}

TEST_F(BleTest, BleConfigBeacon)
{
    uint8_t beacon_payload[SYSHAL_BLE_ADVERTISING_SIZE] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    };
    Init();
    SetBeaconInterval(16);
    SetBeaconPayload(beacon_payload);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_config_beacon(16, beacon_payload));
}

TEST_F(BleTest, BleConfigScanResponse)
{
    uint8_t scan_payload[SYSHAL_BLE_ADVERTISING_SIZE] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
    };
    Init();
    SetScanResponse(scan_payload);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_config_scan_response(scan_payload));
}

TEST_F(BleTest, BleSend)
{
    uint8_t buffer[512];
    for (unsigned int i = 0; i < 512; i++)
        buffer[i] = (uint8_t)i;
    Init();
    SendData(buffer, 64);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_send(buffer, 64));
    ReadIntStatus(NRF52_INT_TX_DATA_SENT);
    ReadTxDataLength(64);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_SEND_COMPLETE, SYSHAL_BLE_NO_ERROR, 64);
}

TEST_F(BleTest, BleReceive)
{
    uint8_t expected_buffer[512];
    uint8_t buffer[512];
    for (unsigned int i = 0; i < 512; i++)
        expected_buffer[i] = (uint8_t)i;
    memset(buffer, 0, sizeof(buffer));
    Init();
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_receive(buffer, 512));
    ReadIntStatus(NRF52_INT_RX_DATA_READY);
    ReadRxDataLength(64);
    ReadRxDataPort(expected_buffer, 64);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_RECEIVE_COMPLETE, SYSHAL_BLE_NO_ERROR, 64);
    EXPECT_ARRAY_EQ(uint8_t, expected_buffer, buffer, 64);
}

TEST_F(BleTest, BleGattConnnectedDisconnected)
{
    Init();
    ReadIntStatus(NRF52_INT_GATT_CONNECTED);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_CONNECTED, SYSHAL_BLE_NO_ERROR, 0);
    ReadIntStatus(0);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_DISCONNECTED, SYSHAL_BLE_NO_ERROR, 0);
    ReadIntStatus(NRF52_INT_GATT_CONNECTED);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_CONNECTED, SYSHAL_BLE_NO_ERROR, 0);
    ReadIntStatus(0);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_DISCONNECTED, SYSHAL_BLE_NO_ERROR, 0);
}

TEST_F(BleTest, BleFwUpgradeComplete)
{
    Init();
    SetMode(SYSHAL_BLE_MODE_FW_UPGRADE);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_set_mode(SYSHAL_BLE_MODE_FW_UPGRADE));
    ReadIntStatus(NRF52_INT_FLASH_PROGRAMMING_DONE);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_FW_UPGRADE_COMPLETE, SYSHAL_BLE_NO_ERROR, 0);
}

TEST_F(BleTest, BleErrorIndication)
{
    Init();
    ReadIntStatus(NRF52_INT_ERROR_INDICATION);
    ReadErrorCode(NRF52_ERROR_CRC);
    EXPECT_EQ(SYSHAL_BLE_NO_ERROR, syshal_ble_tick());
    ExpectEvent(SYSHAL_BLE_EVENT_ERROR_INDICATION, SYSHAL_BLE_ERROR_CRC, 0);
}
