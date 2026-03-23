/*
 * vl53l0x.c — minimal VL53L0X time-of-flight distance sensor driver
 *
 * Communicates via I2C1 (hi2c1, PB6 SCL / PB7 SDA).
 * Default device address: 0x29 (7-bit).
 *
 * Ported from the Pololu VL53L0X Arduino library (MIT licence).
 * Required init sequence and tuning registers are mandated by the ST API.
 */

#include "vl53l0x.h"
#include "i2c.h"

#define VL53L0X_ADDR   (0x29u << 1u)   /* HAL expects 8-bit address */
#define TMO_MS         500u

/* Saved during init, needed to arm each single-shot measurement */
static uint8_t stop_variable;

/* ── Register I/O helpers ──────────────────────────────────────────────── */

static HAL_StatusTypeDef wr8(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, reg, 1u, &val, 1u, TMO_MS);
}

static HAL_StatusTypeDef wr16(uint8_t reg, uint16_t val)
{
    uint8_t b[2] = {(uint8_t)(val >> 8u), (uint8_t)(val & 0xFFu)};
    return HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, reg, 1u, b, 2u, TMO_MS);
}

static HAL_StatusTypeDef rd8(uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, reg, 1u, val, 1u, TMO_MS);
}

static HAL_StatusTypeDef rd16(uint8_t reg, uint16_t *val)
{
    uint8_t b[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, reg, 1u, b, 2u, TMO_MS);
    if (s == HAL_OK) { *val = ((uint16_t)b[0] << 8u) | b[1]; }
    return s;
}

/* ── Mandatory tuning register table (from ST VL53L0X API) ────────────── */

static const uint8_t tuning[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

/* ── SPAD configuration ────────────────────────────────────────────────── */

static int setup_spads(void)
{
    uint8_t val;

    /* Read SPAD count and type from NVM (undocumented access sequence) */
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    wr8(0xFF, 0x06);
    rd8(0x83, &val); wr8(0x83, val | 0x04u);
    wr8(0xFF, 0x07); wr8(0x81, 0x01);
    wr8(0x80, 0x01);
    wr8(0x94, 0x6B); wr8(0x83, 0x00);

    uint32_t t = HAL_GetTick();
    do { rd8(0x83, &val); } while (val == 0x00u && (HAL_GetTick() - t) < TMO_MS);
    if (val == 0x00u) return 0;

    wr8(0x83, 0x01);
    rd8(0x92, &val);
    uint8_t spad_count  = val & 0x7Fu;
    uint8_t is_aperture = (val >> 7u) & 0x01u;

    wr8(0x81, 0x00); wr8(0xFF, 0x06);
    rd8(0x83, &val); wr8(0x83, val & ~0x04u);
    wr8(0xFF, 0x01); wr8(0x00, 0x01);
    wr8(0xFF, 0x00); wr8(0x80, 0x00);

    /* Read the reference SPAD map stored in device NVM */
    uint8_t ref_spad_map[6];
    HAL_I2C_Mem_Read(&hi2c1, VL53L0X_ADDR, 0xB0u, 1u, ref_spad_map, 6u, TMO_MS);

    wr8(0xFF, 0x01); wr8(0x4F, 0x00); wr8(0x4E, 0x2C);
    wr8(0xFF, 0x00); wr8(0xB6, 0xB4);

    /* Re-enable only the first spad_count SPADs of the correct type */
    uint8_t first   = is_aperture ? 12u : 0u;
    uint8_t enabled = 0u;
    for (uint8_t i = 0u; i < 48u; i++) {
        if (i < first || enabled == spad_count) {
            ref_spad_map[i / 8u] &= (uint8_t)~(1u << (i % 8u));
        } else if ((ref_spad_map[i / 8u] >> (i % 8u)) & 0x01u) {
            enabled++;
        }
    }
    HAL_I2C_Mem_Write(&hi2c1, VL53L0X_ADDR, 0xB0u, 1u, ref_spad_map, 6u, TMO_MS);
    return 1;
}

/* ── One calibration step (VHV or phase) ──────────────────────────────── */

static int cal_step(uint8_t vhv_init_byte)
{
    uint8_t val;
    wr8(0x00, 0x01u | vhv_init_byte);

    uint32_t t = HAL_GetTick();
    do { rd8(0x13, &val); } while ((val & 0x07u) == 0u && (HAL_GetTick() - t) < TMO_MS);
    if ((val & 0x07u) == 0u) return 0;

    wr8(0x0B, 0x01);   /* clear interrupt */
    wr8(0x00, 0x00);   /* stop */
    return 1;
}

/* ── Public API ────────────────────────────────────────────────────────── */

int VL53L0X_Init(void)
{
    uint8_t model_id, val;

    /* Verify device presence (MODEL_ID register must read 0xEE) */
    if (rd8(0xC0u, &model_id) != HAL_OK || model_id != 0xEEu) return 0;

    /* Set I2C standard mode */
    wr8(0x88, 0x00);

    /* Data init: save stop_variable needed for single-shot arming */
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    rd8(0x91, &stop_variable);
    wr8(0x00, 0x01); wr8(0xFF, 0x00); wr8(0x80, 0x00);

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    rd8(0x60, &val); wr8(0x60, val | 0x12u);

    /* Signal rate limit: 0.25 MCPS (Q9.7 fixed-point: 0.25 × 128 = 32 = 0x0020) */
    wr16(0x44, 0x0020u);

    /* Enable all sequence steps for calibration */
    wr8(0x01, 0xFF);

    /* Configure SPADs */
    if (!setup_spads()) return 0;

    /* Write mandatory tuning registers */
    for (uint32_t i = 0u; i < sizeof(tuning) / sizeof(tuning[0]); i++) {
        wr8(tuning[i][0], tuning[i][1]);
    }

    /* GPIO: interrupt fires when new sample is ready (active-low) */
    wr8(0x0A, 0x04);
    rd8(0x84, &val);
    if ((val & 0x10u) == 0u) wr8(0x84, val | 0x10u);
    wr8(0x0B, 0x01);   /* clear any pending interrupt */

    /* VHV (voltage high-voltage) calibration */
    wr8(0x01, 0x01);
    if (!cal_step(0x40)) return 0;

    /* Phase calibration */
    wr8(0x01, 0x02);
    if (!cal_step(0x00)) return 0;

    /* Restore sequence config for normal ranging: DSS + PRE_RANGE + FINAL_RANGE */
    wr8(0x01, 0xE8);

    return 1;
}

uint16_t VL53L0X_ReadDistance(void)
{
    uint8_t  val;
    uint16_t range;

    /* Arm the single-shot trigger (requires stop_variable from init) */
    wr8(0x80, 0x01); wr8(0xFF, 0x01); wr8(0x00, 0x00);
    wr8(0x91, stop_variable);
    wr8(0x00, 0x01); wr8(0xFF, 0x00); wr8(0x80, 0x00);

    /* Start single-shot measurement */
    wr8(0x00, 0x01);

    /* Wait for the device to clear the start bit */
    uint32_t t = HAL_GetTick();
    do { rd8(0x00, &val); } while ((val & 0x01u) && (HAL_GetTick() - t) < TMO_MS);
    if (val & 0x01u) return 0xFFFFu;

    /* Wait for result-ready interrupt (bits [2:0] of RESULT_INTERRUPT_STATUS) */
    t = HAL_GetTick();
    do { rd8(0x13, &val); } while ((val & 0x07u) == 0u && (HAL_GetTick() - t) < TMO_MS);
    if ((val & 0x07u) == 0u) return 0xFFFFu;

    /* Read range: bytes 10-11 of RESULT_RANGE_STATUS block (register 0x14 + 10 = 0x1E) */
    if (rd16(0x1Eu, &range) != HAL_OK) return 0xFFFFu;

    /* Clear interrupt */
    wr8(0x0B, 0x01);

    return range;
}
