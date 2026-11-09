#ifndef ZIMANI_CONFIG_H
#define ZIMANI_CONFIG_H
#define ZIMANI_VERSION "3.0.0"
#define HOST_TIMEOUT_US 250000u
#define LINK_TIMEOUT_US 300000u
#define RADIO_PERIOD_US 20000u
#define RADIO_TX_TIMEOUT_US 20000u
#define RADIO_MIN_RX_US 3000u
#define RADIO_CHANNEL 0x45u
#define RADIO_RF_SETUP 0x23u
#define RADIO_RETRIES 0xF3u
#define DEFAULT_P9 2u
/* Captured constants, not established semantic fields. Do not zero them. */
#define ZIMANI_HEADER_BYTES 0xAA,0xF8,0xC4,0x20,0x00,0x43,0x4E,0x1D,0x04
#define ZIMANI_RADIO_ADDRESS 0xF8,0xC4,0x20,0x43,0x4E
#endif
