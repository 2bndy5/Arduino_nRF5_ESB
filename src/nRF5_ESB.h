/*
 * Copyright (C) 2022  Brendan Doherty (2bndy5)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
/**
 * @file nRF5_ESB.h
 * 
 * Header that wraps the nRF5 SDK's ESB implementation into a
 * C++ interface that resembles the RF24 API.
 */
#ifndef __nRF5_ESB_H__
#define __nRF5_ESB_H__
#include <Arduino.h>
#include "nordic/nrf_esb.h"

// workaround "Serial undefined" error.
// see https://github.com/adafruit/Adafruit_nRF52_Arduino/issues/653#issuecomment-853259978
#ifdef USE_TINYUSB
    #include <Adafruit_TinyUSB.h>
#endif

#ifndef PSTR
    #define PSTR(x) (x)
#endif

class nRF5_ESB
{

public:
    // Basic API
    nRF5_ESB();
    bool begin();
    void startListening();
    void stopListening();
    bool available();
    void read(void* buf, uint8_t len);
    bool write(void* buf, uint8_t len, bool multicast = false);
    void openWritingPipe(const uint8_t* address);
    void openReadingPipe(uint8_t pipe, const uint8_t* address);

    // deprecated API
    void openReadingPipe(uint8_t pipe, uint64_t address);
    void openWritingPipe(uint64_t address);

    // Advanced API
    uint32_t failureDetected;
    bool available(uint8_t* pipe_num);
    bool rxFifoFull();
    uint32_t isFifo(bool about_tx);
    bool isFifo(bool about_tx, bool check_empty);
    void powerDown();
    void powerUp();
    bool writeFast(const void* buf, uint8_t len, const bool multicast = false);
    bool writeBlocking(const uint8_t* buff, uint8_t len, uint32_t timeout);
    bool txStandBy();
    bool txStandBy(uint32_t timeout, bool startTx = false);
    bool writeAckPayload(uint8_t pipe, const uint8_t* buf, uint8_t len);
    void startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx = true);
    bool startWrite(const void *buf, uint8_t len, const bool multicast);
    void reuseTx();
    void flush_tx();
    void flush_rx();
    uint8_t sample_ed(void);
    void closeReadingPipe(uint8_t pipe);

    // Configurable API
    void setAddressWidth(uint8_t width);
    void setRetries(uint16_t delay, uint16_t count);
    void setChannel(uint8_t channel);
    uint8_t getChannel();
    void setPayloadSize(uint8_t size);
    uint8_t getPayloadSize();
    uint8_t getDynamicPayloadSize();
    void enableDynamicPayloads();
    void disableDynamicPayloads();
    void enableDynamicAck(void);
    void setAutoAck(bool enable);
    void setAutoAck(uint8_t pipe, bool enable);
    void setPALevel(nrf_esb_tx_power_t level);
    nrf_esb_tx_power_t getPALevel();
    uint32_t getARC();
    bool setDataRate(nrf_esb_bitrate_t speed);
    nrf_esb_bitrate_t getDataRate();
    void setCRCLength(nrf_esb_crc_t length);
    nrf_esb_crc_t getCRCLength();
    void disableCRC();
    void startConstCarrier(nrf_esb_tx_power_t level, uint8_t channel);
    void stopConstCarrier();
    void toggleAllPipes(bool isEnabled);
    void setRadiation(nrf_esb_tx_power_t level, nrf_esb_bitrate_t speed);

protected:
    nrf_esb_config_t nrf_esb_config;
    uint8_t addr_width;
    uint8_t rx_pipes_enabled;
private:
    nrf_esb_payload_t rx_payload;
    nrf_esb_payload_t tx_payload;
};

#endif //__nRF5_ESB_H__
