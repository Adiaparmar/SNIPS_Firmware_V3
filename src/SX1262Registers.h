#pragma once

// ============================================================
//  Trimmed down from RadioLib's SX126x definitions.
//  Only constants actually referenced by SX1262HAL.cpp are kept.
// ============================================================

// SX126X physical layer properties
#define RADIOLIB_SX126X_CRYSTAL_FREQ                            32.0
#define RADIOLIB_SX126X_DIV_EXPONENT                            25

// Max LoRa payload for SNIPS packets
#define SX1262HAL_MAX_PACKET_LEN                                64

// ------------------------------------------------------------
// SX126X SPI commands (opcodes)
// ------------------------------------------------------------

// operational modes commands
#define RADIOLIB_SX126X_CMD_SET_STANDBY                         0x80
#define RADIOLIB_SX126X_CMD_SET_TX                              0x83
#define RADIOLIB_SX126X_CMD_SET_RX                              0x82
#define RADIOLIB_SX126X_CMD_SET_REGULATOR_MODE                  0x96
#define RADIOLIB_SX126X_CMD_CALIBRATE_IMAGE                     0x98
#define RADIOLIB_SX126X_CMD_SET_PA_CONFIG                       0x95

// register and buffer access commands
#define RADIOLIB_SX126X_CMD_WRITE_REGISTER                      0x0D
#define RADIOLIB_SX126X_CMD_READ_REGISTER                       0x1D
#define RADIOLIB_SX126X_CMD_WRITE_BUFFER                        0x0E
#define RADIOLIB_SX126X_CMD_READ_BUFFER                         0x1E

// DIO and IRQ control
#define RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS                  0x08
#define RADIOLIB_SX126X_CMD_GET_IRQ_STATUS                      0x12
#define RADIOLIB_SX126X_CMD_CLEAR_IRQ_STATUS                    0x02
#define RADIOLIB_SX126X_CMD_SET_DIO3_AS_TCXO_CTRL               0x97

// DIO2 RF switch control — Datasheet §13.3.5
// Routes the internal RF front-end switch to DIO2 so the antenna
// connects during TX/RX. CRITICAL for Wio-SX1262 modules.
#define RADIOLIB_SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL          0x9D

// RF, modulation and packet commands
#define RADIOLIB_SX126X_CMD_SET_RF_FREQUENCY                    0x86
#define RADIOLIB_SX126X_CMD_SET_PACKET_TYPE                     0x8A
#define RADIOLIB_SX126X_CMD_SET_TX_PARAMS                       0x8E
#define RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS               0x8B
#define RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS                   0x8C
#define RADIOLIB_SX126X_CMD_SET_BUFFER_BASE_ADDRESS             0x8F

// status commands
#define RADIOLIB_SX126X_CMD_GET_RX_BUFFER_STATUS                0x13
#define RADIOLIB_SX126X_CMD_GET_PACKET_STATUS                   0x14

// ------------------------------------------------------------
// SX126X register map
// ------------------------------------------------------------
#define RADIOLIB_SX126X_REG_LORA_SYNC_WORD_MSB                  0x0740
#define RADIOLIB_SX126X_REG_LORA_SYNC_WORD_LSB                  0x0741
#define RADIOLIB_SX126X_REG_OCP_CONFIGURATION                   0x08E7

// ------------------------------------------------------------
// SX126X SPI command variables
// ------------------------------------------------------------

// RADIOLIB_SX126X_CMD_SET_STANDBY
#define RADIOLIB_SX126X_STANDBY_RC                              0x00

// RADIOLIB_SX126X_CMD_SET_RX
#define RADIOLIB_SX126X_RX_TIMEOUT_INF                          0xFFFFFF

// RADIOLIB_SX126X_CMD_SET_TX
#define RADIOLIB_SX126X_TX_TIMEOUT_NONE                         0x000000

// RADIOLIB_SX126X_CMD_SET_REGULATOR_MODE
#define RADIOLIB_SX126X_REGULATOR_DC_DC                         0x01

// RADIOLIB_SX126X_CMD_CALIBRATE_IMAGE — datasheet Table 9-2
#define RADIOLIB_SX126X_CAL_IMG_430_MHZ_1                       0x6B
#define RADIOLIB_SX126X_CAL_IMG_430_MHZ_2                       0x6F
#define RADIOLIB_SX126X_CAL_IMG_470_MHZ_1                       0x75
#define RADIOLIB_SX126X_CAL_IMG_470_MHZ_2                       0x81
#define RADIOLIB_SX126X_CAL_IMG_779_MHZ_1                       0xC1
#define RADIOLIB_SX126X_CAL_IMG_779_MHZ_2                       0xC5
#define RADIOLIB_SX126X_CAL_IMG_863_MHZ_1                       0xD7
#define RADIOLIB_SX126X_CAL_IMG_863_MHZ_2                       0xDB
#define RADIOLIB_SX126X_CAL_IMG_902_MHZ_1                       0xE1
#define RADIOLIB_SX126X_CAL_IMG_902_MHZ_2                       0xE9

// RADIOLIB_SX126X_CMD_SET_PA_CONFIG
#define RADIOLIB_SX126X_PA_CONFIG_HP_MAX                        0x07
#define RADIOLIB_SX126X_PA_CONFIG_PA_LUT                        0x01
#define RADIOLIB_SX126X_PA_CONFIG_SX1262_8                      0x00

// RADIOLIB_SX126X_CMD_SET_DIO_IRQ_PARAMS
#define RADIOLIB_SX126X_IRQ_TIMEOUT                             0b0000001000000000
#define RADIOLIB_SX126X_IRQ_CRC_ERR                             0b0000000001000000
#define RADIOLIB_SX126X_IRQ_RX_DONE                             0b0000000000000010
#define RADIOLIB_SX126X_IRQ_TX_DONE                             0b0000000000000001
#define RADIOLIB_SX126X_IRQ_ALL                                 0b0100001111111111

// RADIOLIB_SX126X_CMD_SET_DIO3_AS_TCXO_CTRL
#define RADIOLIB_SX126X_DIO3_OUTPUT_1_6                         0x00
#define RADIOLIB_SX126X_DIO3_OUTPUT_1_7                         0x01
#define RADIOLIB_SX126X_DIO3_OUTPUT_1_8                         0x02
#define RADIOLIB_SX126X_DIO3_OUTPUT_2_2                         0x03
#define RADIOLIB_SX126X_DIO3_OUTPUT_2_4                         0x04
#define RADIOLIB_SX126X_DIO3_OUTPUT_2_7                         0x05
#define RADIOLIB_SX126X_DIO3_OUTPUT_3_0                         0x06
#define RADIOLIB_SX126X_DIO3_OUTPUT_3_3                         0x07

// RADIOLIB_SX126X_CMD_SET_PACKET_TYPE
#define RADIOLIB_SX126X_PACKET_TYPE_LORA                        0x01

// RADIOLIB_SX126X_CMD_SET_TX_PARAMS
#define RADIOLIB_SX126X_PA_RAMP_200U                            0x04

// RADIOLIB_SX126X_CMD_SET_MODULATION_PARAMS
#define RADIOLIB_SX126X_LORA_BW_7_8                             0x00
#define RADIOLIB_SX126X_LORA_BW_10_4                            0x08
#define RADIOLIB_SX126X_LORA_BW_15_6                            0x01
#define RADIOLIB_SX126X_LORA_BW_20_8                            0x09
#define RADIOLIB_SX126X_LORA_BW_31_25                           0x02
#define RADIOLIB_SX126X_LORA_BW_41_7                            0x0A
#define RADIOLIB_SX126X_LORA_BW_62_5                            0x03
#define RADIOLIB_SX126X_LORA_BW_125_0                           0x04
#define RADIOLIB_SX126X_LORA_BW_250_0                           0x05
#define RADIOLIB_SX126X_LORA_BW_500_0                           0x06
#define RADIOLIB_SX126X_LORA_CR_4_5                             0x01
#define RADIOLIB_SX126X_LORA_CR_4_6                             0x02
#define RADIOLIB_SX126X_LORA_CR_4_7                             0x03
#define RADIOLIB_SX126X_LORA_CR_4_8                             0x04
#define RADIOLIB_SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF         0x00

// RADIOLIB_SX126X_CMD_SET_PACKET_PARAMS
#define RADIOLIB_SX126X_LORA_HEADER_EXPLICIT                    0x00
#define RADIOLIB_SX126X_LORA_CRC_ON                             0x01
#define RADIOLIB_SX126X_LORA_IQ_STANDARD                        0x00

// RADIOLIB_SX126X_CMD_GET_STATUS
#define RADIOLIB_SX126X_STATUS_CMD_FAILED                       0b00001010

// LoRa sync words (private network)
#define RADIOLIB_SX126X_SYNC_WORD_PRIVATE                       0x12

// ------------------------------------------------------------
// Our own error codes
// ------------------------------------------------------------
#define RADIOLIB_ERR_NONE              0
#define RADIOLIB_ERR_SPI_CMD_FAILED   -1
