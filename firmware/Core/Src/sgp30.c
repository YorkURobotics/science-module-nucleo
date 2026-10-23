/*
 * sgp30.c — Implementation of the SGP30 driver.
 *
 * See sgp30.h for the public API and usage. This file contains the
 * full implementation: I²C command set, CRC-8, HAL transport, and
 * peripheral reset on transport failure.
 *
 * Authored by Nathan Cheung <ncheung3@my.yorku.ca>
 * York University Robotics Society (YURS) — Electrical Embedded Team
 * Mars Rover Science Module
 * Date: 21 May 2026
 */

#include "sgp30.h"


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

/* 7-bit address 0x58 (fixed in silicon), pre-shifted as STM32 HAL
 * expects. The shift leaves bit 0 free for HAL to set the R/W bit. */
#define SGP30_I2C_ADDR_SHIFTED  ((0x58u) << 1)

/* Command opcodes — SGP30 datasheet v1.0 May 2020 table 10. */
#define CMD_INIT_AIR_QUALITY    0x2003u
#define CMD_MEASURE_IAQ         0x2008u
#define CMD_GET_BASELINE        0x2015u
#define CMD_SET_BASELINE        0x201Eu
#define CMD_MEASURE_RAW         0x2050u
#define CMD_MEASURE_SELF_TEST   0x2032u
#define CMD_GET_SERIAL_ID       0x3682u

/* Datasheet maximum execution times. We always use the max (not typical);
 * a few extra ms per call is invisible at 1 Hz cadence. */
#define DELAY_INIT_MS           10u
#define DELAY_MEASURE_IAQ_MS    12u
#define DELAY_GET_BASELINE_MS   10u
#define DELAY_SET_BASELINE_MS   10u
#define DELAY_MEASURE_RAW_MS    25u
#define DELAY_SELF_TEST_MS      220u
#define DELAY_GET_SERIAL_ID_MS  1u

/* 16-bit data pattern the sensor returns on a passing self-test. */
#define SELF_TEST_PASS_PATTERN  0xD400u

/* CRC-8 parameters from SGP30 datasheet table 13: polynomial 0x31,
 * init 0xFF, no reflection, no final XOR. Datasheet test vector:
 * CRC(0xBE, 0xEF) = 0x92. */
#define CRC8_POLYNOMIAL         0x31u
#define CRC8_INIT               0xFFu

/* I²C timeout. Self-test is the longest command (~220 ms); 500 ms gives
 * headroom for clock-stretching while still failing fast on a dead bus. */
#define I2C_TIMEOUT_MS          500u


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/* Compute the CRC-8 checksum of `n` bytes using the SGP30 parameters.
 * Called to verify received data and to append CRC bytes to outgoing
 * data. Bit-by-bit shift-register implementation. */
static uint8_t crc8(const uint8_t *data, uint8_t n)
{
    uint8_t crc = CRC8_INIT;

    for (uint8_t i = 0; i < n; i++) {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLYNOMIAL);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}


/* Reset the I²C peripheral to a known-good state after any HAL
 * transport failure. See sgp30.h for the design rationale. */
static void sgp30_reset_peripheral(I2C_HandleTypeDef *hi2c)
{
    (void)HAL_I2C_DeInit(hi2c);
    (void)HAL_I2C_Init(hi2c);
}


/* Send a two-byte command to the SGP30, big-endian on the wire.
 * Commands without arguments carry no CRC; only data words do. On HAL
 * failure, resets the peripheral and propagates the HAL status. */
static HAL_StatusTypeDef send_command(I2C_HandleTypeDef *hi2c, uint16_t cmd)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(cmd >> 8);         /* High byte first. */
    buf[1] = (uint8_t)(cmd & 0xFFu);      /* Low byte second. */

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        hi2c, SGP30_I2C_ADDR_SHIFTED, buf, sizeof(buf), I2C_TIMEOUT_MS);

    if (status != HAL_OK) {
        sgp30_reset_peripheral(hi2c);
    }
    return status;
}


/* Read `n_words` 16-bit words from the SGP30 into `out`. Each word
 * arrives as [data_hi, data_lo, CRC]; we verify each per-word CRC and
 * reassemble into host-native uint16. Bounded at 3 words (the longest
 * transaction is the 48-bit serial-ID read). Returns HAL_ERROR on
 * invalid n_words or CRC mismatch (caller must discard partial results
 * in `out`); propagates the HAL status on transport failure (and resets
 * the peripheral). */
static HAL_StatusTypeDef read_words(I2C_HandleTypeDef *hi2c,
                                    uint16_t *out, uint8_t n_words)
{
    if (n_words == 0u || n_words > 3u) {
        return HAL_ERROR;
    }

    uint8_t buf[3u * 3u];                          /* 3 words × 3 bytes per word. */
    uint16_t bytes_to_read = (uint16_t)(n_words * 3u);

    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
        hi2c, SGP30_I2C_ADDR_SHIFTED, buf, bytes_to_read, I2C_TIMEOUT_MS);

    if (status != HAL_OK) {
        sgp30_reset_peripheral(hi2c);
        return status;
    }

    for (uint8_t i = 0; i < n_words; i++) {
        const uint8_t *word_bytes = &buf[i * 3u];

        /* Verify the CRC byte that follows each word's two data bytes. */
        if (crc8(word_bytes, 2) != word_bytes[2]) {
            return HAL_ERROR;
        }

        /* Reassemble the two big-endian bytes into a host-native uint16. */
        out[i] = (uint16_t)((word_bytes[0] << 8) | word_bytes[1]);
    }

    return HAL_OK;
}


/* Combined command-then-response: send `cmd`, wait `delay_ms` (the
 * datasheet's max compute time for that command), then read the
 * response. Used by every public read function. */
static HAL_StatusTypeDef send_command_and_read(I2C_HandleTypeDef *hi2c,
                                               uint16_t cmd,
                                               uint32_t delay_ms,
                                               uint16_t *out_words,
                                               uint8_t n_words)
{
    HAL_StatusTypeDef status = send_command(hi2c, cmd);
    if (status != HAL_OK) {
        return status;
    }
    HAL_Delay(delay_ms);
    return read_words(hi2c, out_words, n_words);
}


/* Common entry guard: rejects NULL `dev` and uninitialised dev (no
 * stored hi2c handle). Returns HAL_ERROR in either case. */
static HAL_StatusTypeDef check_dev(sgp30_t *dev)
{
    if (dev == NULL || dev->hi2c == NULL) {
        return HAL_ERROR;
    }
    return HAL_OK;
}


/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef sgp30_init(sgp30_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    dev->hi2c = hi2c;

    HAL_StatusTypeDef status = send_command(hi2c, CMD_INIT_AIR_QUALITY);
    if (status == HAL_OK) {
        HAL_Delay(DELAY_INIT_MS);
    }
    return status;
}


HAL_StatusTypeDef sgp30_read_iaq(sgp30_t *dev, sgp30_iaq_t *out)
{
    if (check_dev(dev) != HAL_OK || out == NULL) {
        return HAL_ERROR;
    }

    uint16_t words[2];
    HAL_StatusTypeDef status = send_command_and_read(
        dev->hi2c, CMD_MEASURE_IAQ, DELAY_MEASURE_IAQ_MS, words, 2);
    if (status == HAL_OK) {
        /* Wire order: words[0] = CO2eq, words[1] = TVOC. */
        out->co2eq_ppm = words[0];
        out->tvoc_ppb  = words[1];
    }
    return status;
}


HAL_StatusTypeDef sgp30_self_test(sgp30_t *dev)
{
    if (check_dev(dev) != HAL_OK) {
        return HAL_ERROR;
    }

    uint16_t result;
    HAL_StatusTypeDef status = send_command_and_read(
        dev->hi2c, CMD_MEASURE_SELF_TEST, DELAY_SELF_TEST_MS, &result, 1);
    if (status == HAL_OK) {
        /* Pass = 0xD400 (datasheet §6, p10). The pattern was picked
         * not to look like a stuck-bus result (0x0000 or 0xFFFF). */
        if (result != SELF_TEST_PASS_PATTERN) {
            status = HAL_ERROR;
        }
    }
    return status;
}


HAL_StatusTypeDef sgp30_read_raw(sgp30_t *dev, sgp30_raw_t *out)
{
    if (check_dev(dev) != HAL_OK || out == NULL) {
        return HAL_ERROR;
    }

    uint16_t words[2];
    HAL_StatusTypeDef status = send_command_and_read(
        dev->hi2c, CMD_MEASURE_RAW, DELAY_MEASURE_RAW_MS, words, 2);
    if (status == HAL_OK) {
        /* Wire order: words[0] = H₂, words[1] = ethanol. */
        out->h2_raw      = words[0];
        out->ethanol_raw = words[1];
    }
    return status;
}


HAL_StatusTypeDef sgp30_get_serial_id(sgp30_t *dev, uint64_t *serial_id)
{
    if (check_dev(dev) != HAL_OK || serial_id == NULL) {
        return HAL_ERROR;
    }

    uint16_t words[3];
    HAL_StatusTypeDef status = send_command_and_read(
        dev->hi2c, CMD_GET_SERIAL_ID, DELAY_GET_SERIAL_ID_MS, words, 3);
    if (status == HAL_OK) {
        /* words[0] is most-significant. The (uint64_t) casts are
         * required — shifting a uint16_t by 32 is undefined behaviour. */
        *serial_id = ((uint64_t)words[0] << 32)
                   | ((uint64_t)words[1] << 16)
                   | ((uint64_t)words[2] <<  0);
    }
    return status;
}


HAL_StatusTypeDef sgp30_get_baseline(sgp30_t *dev, sgp30_baseline_t *out)
{
    if (check_dev(dev) != HAL_OK || out == NULL) {
        return HAL_ERROR;
    }

    uint16_t words[2];
    HAL_StatusTypeDef status = send_command_and_read(
        dev->hi2c, CMD_GET_BASELINE, DELAY_GET_BASELINE_MS, words, 2);
    if (status == HAL_OK) {
        /* GET wire order: words[0] = CO2eq, words[1] = TVOC. SET wants
         * the OPPOSITE order; the named struct fields hide that quirk
         * from callers. */
        out->co2eq_baseline = words[0];
        out->tvoc_baseline  = words[1];

        /* Both-zero means the sensor has not stabilised (~60 min after
         * init). Reject — persisting a zero baseline would degrade
         * accuracy on the next restore. */
        if (out->co2eq_baseline == 0u && out->tvoc_baseline == 0u) {
            status = HAL_ERROR;
        }
    }
    return status;
}


HAL_StatusTypeDef sgp30_set_baseline(sgp30_t *dev, const sgp30_baseline_t *in)
{
    if (check_dev(dev) != HAL_OK || in == NULL) {
        return HAL_ERROR;
    }
    /* Reject all-zero baselines (unpopulated persistence slot). */
    if (in->co2eq_baseline == 0u && in->tvoc_baseline == 0u) {
        return HAL_ERROR;
    }

    /* Wire layout (8 bytes total):
     *   buf[0..1] command id (big-endian)
     *   buf[2..3] TVOC baseline (big-endian) — TVOC precedes CO2eq
     *   buf[4]    CRC over buf[2..3]
     *   buf[5..6] CO2eq baseline (big-endian)
     *   buf[7]    CRC over buf[5..6]
     * The TVOC-first order is REVERSED from GET-baseline's wire order.
     * Chip-level asymmetry; the ordering must be preserved. */
    uint8_t buf[8];

    buf[0] = (uint8_t)(CMD_SET_BASELINE >> 8);
    buf[1] = (uint8_t)(CMD_SET_BASELINE & 0xFFu);

    buf[2] = (uint8_t)(in->tvoc_baseline >> 8);
    buf[3] = (uint8_t)(in->tvoc_baseline & 0xFFu);
    buf[4] = crc8(&buf[2], 2);

    buf[5] = (uint8_t)(in->co2eq_baseline >> 8);
    buf[6] = (uint8_t)(in->co2eq_baseline & 0xFFu);
    buf[7] = crc8(&buf[5], 2);

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        dev->hi2c, SGP30_I2C_ADDR_SHIFTED, buf, sizeof(buf), I2C_TIMEOUT_MS);

    if (status == HAL_OK) {
        HAL_Delay(DELAY_SET_BASELINE_MS);
    } else {
        sgp30_reset_peripheral(dev->hi2c);
    }
    return status;
}
