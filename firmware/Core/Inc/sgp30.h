/*
 * sgp30.h — Driver for the Sensirion SGP30 air-quality sensor.
 *
 * The SGP30 reports two air-quality values over I²C at address 0x58:
 *
 *     - TVOC (total volatile organic compounds), ppb.
 *     - eCO2 (CO2-equivalent), ppm.
 *
 * eCO2 is NOT a true CO2 reading. The SGP30 measures hydrogen-gas
 * concentration and computes an equivalent CO2 via a Sensirion-proprietary
 * correlation. For accurate CO2, use a dedicated NDIR sensor.
 *
 * Usage:
 *     sgp30_t sgp30;
 *     sgp30_init(&sgp30, &hi2c1);      // once at startup
 *     // ...wait 15 s for sensor warm-up...
 *     sgp30_iaq_t r;
 *     if (sgp30_read_iaq(&sgp30, &r) == HAL_OK) {
 *         // use r.co2eq_ppm, r.tvoc_ppb
 *     }
 *
 * Read no faster than 1 Hz — the sensor's dynamic-baseline algorithm
 * assumes that cadence and produces drift otherwise.
 *
 * Blocking durations (longest single call is the self-test at ~220 ms):
 *     init             ~10 ms       self_test       ~220 ms
 *     read_iaq         ~12 ms       read_raw         ~25 ms
 *     get/set_baseline ~10 ms       get_serial_id    ~1 ms
 *
 * Concurrency
 * -----------
 * Not thread-safe. Every public-function call uses HAL_Delay (a
 * busy-wait) for sensor waits. Concurrent calls from multiple FreeRTOS
 * tasks would corrupt each other's I²C transactions — if the
 * application has more than one caller, the application must serialize
 * them externally.
 *
 * NOT safe from ISR context. Call driver functions only from the main
 * loop or from a task.
 *
 * Failure handling
 * ----------------
 * A failed HAL transaction returns the HAL's status (HAL_ERROR or
 * HAL_TIMEOUT) and resets the I²C peripheral (HAL_I2C_DeInit +
 * HAL_I2C_Init). The driver has no give-up state, no retry counter,
 * no external-trigger requirement — every call auto-retries on a
 * fresh peripheral, so the driver recovers as soon as the underlying
 * fault clears. Faults a peripheral reset cannot clear (dead slave,
 * snapped trace) are the application's responsibility — typically a
 * watchdog-driven MCU reset.
 *
 * Authored by Nathan Cheung <ncheung3@my.yorku.ca>
 * York University Robotics Society (YURS) — Electrical Embedded Team
 * Mars Rover Science Module
 * Date: 21 May 2026
 */

#ifndef SGP30_H
#define SGP30_H

#include <stdint.h>

#include "main.h"


/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

/* Per-instance state. The caller allocates one of these and passes a
 * pointer to every driver function. `hi2c` is set by sgp30_init() and
 * not modified afterward; the application should treat the struct as
 * opaque (don't access fields directly). */
typedef struct {
    I2C_HandleTypeDef *hi2c;
} sgp30_t;

typedef struct {
    uint16_t co2eq_ppm;   /* CO2-equivalent, ppm. */
    uint16_t tvoc_ppb;    /* TVOC, ppb.            */
} sgp30_iaq_t;

typedef struct {
    uint16_t h2_raw;       /* Raw H₂ signal (arbitrary units, trend-only). */
    uint16_t ethanol_raw;  /* Raw ethanol signal (arbitrary units).         */
} sgp30_raw_t;

/* The sensor's wire format transmits the two fields below in OPPOSITE
 * order between get-baseline and set-baseline (a chip-level quirk). The
 * driver hides that asymmetry: callers always read and write by field
 * name, and a get → save → restore → set round-trip preserves values. */
typedef struct {
    uint16_t co2eq_baseline;
    uint16_t tvoc_baseline;
} sgp30_baseline_t;


/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/* Initialise the driver. Stores hi2c in dev->hi2c and sends the sensor's
 * INIT_AIR_QUALITY command. After return, wait 15 s — the sensor returns
 * fixed defaults during warm-up. Returns HAL_ERROR on NULL dev or NULL
 * hi2c; HAL_ERROR/HAL_TIMEOUT propagated from the I²C transmit on
 * sensor NACK. */
HAL_StatusTypeDef sgp30_init(sgp30_t *dev, I2C_HandleTypeDef *hi2c);

/* Read one IAQ measurement. Call no faster than 1 Hz. Returns HAL_ERROR
 * on NULL dev/out, uninitialised dev (hi2c == NULL), CRC mismatch;
 * HAL_ERROR/HAL_TIMEOUT propagated from the I²C transport on transmit
 * or receive failure. */
HAL_StatusTypeDef sgp30_read_iaq(sgp30_t *dev, sgp30_iaq_t *out);

/* Run the sensor's on-chip self-test. Use during hardware bring-up only.
 * Do NOT call during the 15 s warm-up after sgp30_init — mixing self-test
 * with measurement mode requires a save/restore sequence the driver
 * does not implement. Returns HAL_ERROR if the self-test pattern does
 * not match 0xD400, otherwise propagates the HAL transport status. */
HAL_StatusTypeDef sgp30_self_test(sgp30_t *dev);

/* Read the raw H₂ and ethanol signals (pre-IAQ-algorithm). Useful for
 * debugging unexpected eCO2/TVOC values. */
HAL_StatusTypeDef sgp30_read_raw(sgp30_t *dev, sgp30_raw_t *out);

/* Retrieve the SGP30's 48-bit factory serial (low 48 bits of result;
 * upper 16 bits zero). Useful for log traceability and detecting sensor
 * swaps. */
HAL_StatusTypeDef sgp30_get_serial_id(sgp30_t *dev, uint64_t *serial_id);

/* Read the current dynamic-baseline values. Returns HAL_ERROR if both
 * are zero (sensor not yet stabilised — retry periodically; baselines
 * become valid ~60 min after init). Persistence pattern: save once per
 * hour to non-volatile storage and at clean shutdown; restore via
 * sgp30_set_baseline at the next boot, BEFORE any read_iaq calls. */
HAL_StatusTypeDef sgp30_get_baseline(sgp30_t *dev, sgp30_baseline_t *out);

/* Restore a previously saved baseline. Call shortly after sgp30_init and
 * BEFORE any read_iaq. Discard baselines older than 7 days — stale ones
 * degrade accuracy rather than improving it. Returns HAL_ERROR on NULL
 * or all-zero input (treated as an unpopulated persistence slot). */
HAL_StatusTypeDef sgp30_set_baseline(sgp30_t *dev, const sgp30_baseline_t *in);


#endif /* SGP30_H */
