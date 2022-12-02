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
 * @file nRF52x_ESB.cpp
 * 
 * Implementation file for the nRF5_ESB.h
 */
#include "nRF5_ESB.h"

volatile bool nrf_esb_rx_ready = false;
volatile bool nrf_esb_tx_sent = false;
volatile bool nrf_esb_tx_fail = false;

void nrf_esb_event_handler(nrf_esb_evt_t const* p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            nrf_esb_tx_sent = true;
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            nrf_esb_tx_fail = true;
            (void)nrf_esb_flush_tx();
            (void)nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            nrf_esb_rx_ready = true;
            break;
    } // NOTE: no default case as all defined events are handled above
}

nRF5_ESB::nRF5_ESB() : failureDetected(NRF_SUCCESS), addr_width(5), rx_pipes_enabled(3)
{
    tx_payload.length = NRF_ESB_MAX_PAYLOAD_LENGTH;
    tx_payload.pipe = 0;
    tx_payload.rssi = 0;
    tx_payload.noack = false;
    tx_payload.pid = 0;
    memset(&tx_payload.data, 0, NRF_ESB_MAX_PAYLOAD_LENGTH);

    rx_payload.length = NRF_ESB_MAX_PAYLOAD_LENGTH;
    rx_payload.pipe = 0;
    rx_payload.rssi = 0;
    rx_payload.noack = false;
    rx_payload.pid = 0;
    memset(&rx_payload.data, 0, NRF_ESB_MAX_PAYLOAD_LENGTH);

    nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB;
    nrf_esb_config.mode = NRF_ESB_MODE_PTX;
    nrf_esb_config.event_handler = nrf_esb_event_handler;
    nrf_esb_config.bitrate = NRF_ESB_BITRATE_1MBPS;
}

void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
        // Wait for the external oscillator to start up
    }

    NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0) {
        // Do nothing.
    }
}

bool nRF5_ESB::begin()
{
    clock_initialization();
    NRF_RADIO->POWER = 1;
    NRF_POWER->DCDCEN = 1;

    failureDetected = nrf_esb_init(&nrf_esb_config);
    if (failureDetected) {
        return false;
    }

    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    failureDetected = nrf_esb_set_base_address_0(base_addr_0);
    if (failureDetected) {
        return false;
    }

    failureDetected = nrf_esb_set_base_address_1(base_addr_1);
    if (failureDetected) {
        return false;
    }

    failureDetected = nrf_esb_set_prefixes(addr_prefix, 3); // only enable pipes 0 & 1
    if (failureDetected) {
        return false;
    }

    setChannel(76); // use default from RF24 API
    return !failureDetected;
}

void nRF5_ESB::startListening()
{
    failureDetected = nrf_esb_start_rx();
}

void nRF5_ESB::stopListening()
{
    failureDetected = nrf_esb_stop_rx();
}

bool nRF5_ESB::available()
{
    return nrf_esb_get_fifo_occupancy(false) > 0;
}

bool nRF5_ESB::available(uint8_t* pipe_num)
{
    if (available()) {
        failureDetected = nrf_esb_peek_rx_fifo(&rx_payload);
        *pipe_num = rx_payload.pipe;
        return true;
    }
    return false;
}

void nRF5_ESB::read(void* buf, uint8_t len)
{
    failureDetected = nrf_esb_read_rx_payload(&rx_payload);
    if (!failureDetected) {
        // TODO
        // 1. if len != rx_payload.length { don't pop from RX FIFO }
        // 2. allow 0 length reads to simply pop payload from FIFO?
        memcpy(buf, &rx_payload.data, len);
    }
    nrf_esb_rx_ready = nrf_esb_get_fifo_occupancy(false) > 0;
}

bool nRF5_ESB::write(void* buf, uint8_t len, bool multicast)
{
    nrf_esb_config.tx_mode = NRF_ESB_TXMODE_AUTO;
    tx_payload.noack = multicast;
    failureDetected = nrf_esb_write_payload(&tx_payload);
    return failureDetected == NRF_SUCCESS;
}

void nRF5_ESB::openReadingPipe(uint8_t pipe, const uint8_t* address)
{
    if (pipe)
        failureDetected = nrf_esb_set_base_address_1(address);
    else
        failureDetected = nrf_esb_set_base_address_0(address);

    if (!failureDetected)
        failureDetected = nrf_esb_update_prefix(pipe, address[addr_width - 1]);
    
    if (!failureDetected) {
        rx_pipes_enabled |= static_cast<uint8_t>(1 << pipe);
        failureDetected = nrf_esb_enable_pipes(rx_pipes_enabled);
    }
}

void nRF5_ESB::openWritingPipe(const uint8_t* address)
{
    failureDetected = nrf_esb_set_base_address_0(address);

    if (!failureDetected)
        failureDetected = nrf_esb_update_prefix(0, address[addr_width - 1]);

    if (!failureDetected) {
        rx_pipes_enabled |= static_cast<uint8_t>(1);
        failureDetected = nrf_esb_enable_pipes(rx_pipes_enabled);
    }
}

void nRF5_ESB::openReadingPipe(uint8_t pipe, uint64_t address)
{
    openReadingPipe(pipe, reinterpret_cast<uint8_t*>(&address));
}

void nRF5_ESB::openWritingPipe(uint64_t address)
{
    openWritingPipe(reinterpret_cast<uint8_t*>(&address));
}

void nRF5_ESB::closeReadingPipe(uint8_t pipe)
{
    rx_pipes_enabled &= ~static_cast<uint8_t>(1 << pipe);
    failureDetected = nrf_esb_enable_pipes(rx_pipes_enabled);
}

bool nRF5_ESB::rxFifoFull()
{
    return nrf_esb_get_fifo_occupancy(false) == NRF_ESB_RX_FIFO_SIZE;
}

uint32_t nRF5_ESB::isFifo(bool about_tx)
{
    return nrf_esb_get_fifo_occupancy(about_tx);
}

bool nRF5_ESB::isFifo(bool about_tx, bool check_empty)
{
    uint32_t count = isFifo(about_tx);
    // is fifo empty?
    if (check_empty)
        return count == 0;
    // is fifo full?
    if (about_tx)
        return count == NRF_ESB_TX_FIFO_SIZE;
    return count == NRF_ESB_RX_FIFO_SIZE;
}

void nRF5_ESB::powerDown()
{
    // TODO: nrf_esb_suspend() only relaxes the protocol usage on the radio peripheral.
    // Powering down the radio peripheral may require manipulation of registers/state machine.
    failureDetected = nrf_esb_suspend();
}

void nRF5_ESB::powerUp()
{
    // TODO: nrf_esb_init() may be too costly here.
    // Maybe manipulate the radio peripheral directly instead.
    failureDetected = nrf_esb_init(&nrf_esb_config);
}

bool nRF5_ESB::writeFast(const void* buf, uint8_t len, const bool multicast)
{
    nrf_esb_config.tx_mode = NRF_ESB_TXMODE_MANUAL_START;
    tx_payload.noack = multicast;
    uint32_t err_code = nrf_esb_write_payload(&tx_payload);
    return err_code == NRF_SUCCESS;
}

bool nRF5_ESB::writeBlocking(const uint8_t* buff, uint8_t len, uint32_t timeout)
{
    return 1;
}

bool nRF5_ESB::txStandBy()
{
    return 1;
}

bool nRF5_ESB::txStandBy(uint32_t timeout, bool startTx)
{
    return 1;
}

bool nRF5_ESB::writeAckPayload(uint8_t pipe, const uint8_t* buf, uint8_t len)
{
    // TODO: Assert the radio is not in TX mode before doing this.
    tx_payload.pipe = pipe;
    memcpy(tx_payload.data, buf, len);
    tx_payload.length = len;
    return nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS;
}

void nRF5_ESB::startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx)
{
    tx_payload.noack = multicast;
    memcpy(tx_payload.data, buf, len);
    tx_payload.length = len;
    nrf_esb_config.tx_mode = NRF_ESB_TXMODE_MANUAL;
    failureDetected = nrf_esb_write_payload(&tx_payload);
    if (startTx)
        failureDetected = nrf_esb_start_tx();
}

// NOTE: nRF5 ESB implementation already uses interrupts,
// so this function would seem equivalent to startFastWrite()
bool nRF5_ESB::startWrite(const void* buf, uint8_t len, const bool multicast)
{
    startFastWrite(buf, len, multicast, true);
    return !failureDetected;
}

void nRF5_ESB::reuseTx()
{
    // TODO: This should be more involved as compared to RF24 API.
    nrf_esb_reuse_pid(0); // I wish I could be this lazy
}

void nRF5_ESB::flush_tx()
{
    failureDetected = nrf_esb_flush_tx();
}

void nRF5_ESB::flush_rx()
{
    failureDetected = nrf_esb_flush_rx();
}

#define ED_RSSISCALE 4 // From electrical specifications
uint8_t nRF5_ESB::sample_ed(void)
{
    int val;
    NRF_RADIO->TASKS_EDSTART = 1; // Start
    while (NRF_RADIO->EVENTS_EDEND != 1) {
        // CPU can sleep here or do something else
        // Use of interrupts are encouraged
    }
    val = NRF_RADIO->EDSAMPLE;                             // Read level
    return (uint8_t)(val > 63 ? 255 : val * ED_RSSISCALE); // Convert to IEEE 802.15.4 scale
}

void nRF5_ESB::setAddressWidth(uint8_t width)
{
    addr_width = width;
    failureDetected = nrf_esb_set_address_length(width);
}

void nRF5_ESB::setRetries(uint16_t delay, uint16_t count)
{
    if (nrf_esb_set_retransmit_delay(delay) == NRF_SUCCESS)
        failureDetected = nrf_esb_set_retransmit_count(count);
}

void nRF5_ESB::setChannel(uint8_t channel)
{
    failureDetected = nrf_esb_set_rf_channel(static_cast<uint32_t>(channel));
    // NOTE: we could return the error code, but the RF24 API doesn't.
}

uint8_t nRF5_ESB::getChannel()
{
    uint32_t channel;
    failureDetected = nrf_esb_get_rf_channel(&channel);
    // TODO: we should return an invalid value if (error code != NRF_SUCCESS)
    return static_cast<uint8_t>(channel);
}

void nRF5_ESB::setPayloadSize(uint8_t size)
{
    // TODO: possibly should also apply to all payloads in the FIFOs
    nrf_esb_config.payload_length = size;
    // NOTE: update_rf_payload_format_esb() is not exposed in public API.
    // use set_address_length() to invoke instead.
    nrf_esb_set_address_length(addr_width);
}

uint8_t nRF5_ESB::getPayloadSize()
{
    return nrf_esb_config.payload_length;
}

uint8_t nRF5_ESB::getDynamicPayloadSize()
{
    failureDetected = nrf_esb_peek_rx_fifo(&rx_payload);
    if (!failureDetected)
        return rx_payload.length;
    return static_cast<uint8_t>(0);
}

void nRF5_ESB::enableDynamicPayloads()
{
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
    failureDetected = !nrf_esb_update_protocol();
}

void nRF5_ESB::disableDynamicPayloads()
{
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB;
    failureDetected = !nrf_esb_update_protocol();
}

void nRF5_ESB::enableDynamicAck(void)
{
    // NOTE: This could probably be set in begin() since we're not
    // compensating for cloned nRF24L01 modules.
    nrf_esb_config.selective_auto_ack = true;
}

void nRF5_ESB::setAutoAck(bool enable)
{
    // TODO: possibly should apply to payloads' noack flags in the TX FIFO
    if (!enable) {
        tx_payload.noack = false;
    }
    // ESB protocol implementation in nRF5 SDK seems to
    // require dynamic payloads for auto-ack functionality to work
    if (enable) {
        // TODO: find a way to make this OTA compatible with RF24 API
        enableDynamicPayloads();
    }
}

void nRF5_ESB::setAutoAck(uint8_t pipe, bool enable)
{
    // TODO: possibly should be applied to the payloads' noack flags in
    // the TX FIFO because this function is geared towards individual pipes.
    setAutoAck(enable); // treat all pipes the same for now
}

void nRF5_ESB::setPALevel(nrf_esb_tx_power_t level)
{
    failureDetected = nrf_esb_set_tx_power(level);
}

nrf_esb_tx_power_t nRF5_ESB::getPALevel()
{
    return nrf_esb_config.tx_output_power;
}

uint32_t nRF5_ESB::getARC()
{
    return nrf_esb_get_last_arc();
}

bool nRF5_ESB::setDataRate(nrf_esb_bitrate_t speed)
{
    return nrf_esb_set_bitrate(speed) == NRF_SUCCESS;
}

nrf_esb_bitrate_t nRF5_ESB::getDataRate()
{
    return nrf_esb_config.bitrate;
}

void nRF5_ESB::setCRCLength(nrf_esb_crc_t length)
{
    nrf_esb_config.crc = length;
    switch(nrf_esb_config.crc)
    {
        case NRF_ESB_CRC_16BIT:
            NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
            NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
            break;

        case NRF_ESB_CRC_8BIT:
            NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
            NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
            break;

        case NRF_ESB_CRC_OFF:
            NRF_RADIO->CRCINIT = 0x00UL;
            NRF_RADIO->CRCPOLY = 0x00UL;
            break;

        default:
            return;
    }
    NRF_RADIO->CRCCNF = nrf_esb_config.crc << RADIO_CRCCNF_LEN_Pos;
}

nrf_esb_crc_t nRF5_ESB::getCRCLength()
{
    return nrf_esb_config.crc;
}

void nRF5_ESB::disableCRC()
{
    setCRCLength(NRF_ESB_CRC_OFF);
}

void nRF5_ESB::startConstCarrier(nrf_esb_tx_power_t level, uint8_t channel)
{
}

void nRF5_ESB::stopConstCarrier()
{
}

void nRF5_ESB::toggleAllPipes(bool isEnabled)
{
    failureDetected = nrf_esb_enable_pipes(isEnabled * 0xFF);
}

void nRF5_ESB::setRadiation(nrf_esb_tx_power_t level, nrf_esb_bitrate_t speed)
{
    setPALevel(level);
    setDataRate(speed);
}
