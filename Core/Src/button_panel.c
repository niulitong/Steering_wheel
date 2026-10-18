/**
 * @file    button_panel.c
 * @brief   Steering wheel button panel implementation.
 *
 * Signal sources (all active-low with internal pull-up):
 *   PB12  KEY1  – Recorder/camera toggle (rising edge → TX)
 *   PB13  KEY2  – Error-clear trigger (rising edge → TX)
 *   PA6   S1    – Rotary pos.0 → drive mode S
 *   PA7   S2    – Rotary pos.1 → drive mode Q
 *   PB0   S3    – Rotary pos.2 → drive mode C
 *   PB1   S4    – Rotary pos.3 → drive mode E
 *   PA1   ADC1  – Potentiometer → 8-level slip rate
 *
 * CAN output (event-driven):
 *   ID 0x700 — slip level (0..7), sent on change
 *   ID 0x701 — recorder/camera toggle (bit0 = 1), sent on PB12 rising edge
 *   ID 0x702 — error clear (bit0 = 1), sent on PB13 rising edge
 *   ID 0x310 — drive mode request (vehicle protocol), sent on change
 */
#include "button_panel.h"
#include "can.h"

/* ========================================================================= */
/* Module-local types                                                        */
/* ========================================================================= */
typedef struct {
    uint8_t  stable;            /* 1 = debounced state, 0 = still bouncing    */
    uint8_t  level;             /* current stable level (0=pressed,1=idle)    */
    uint32_t cnt;               /* consecutive consistent-sample counter      */
} btn_debounce_t;

/* ========================================================================= */
/* Module-local variables                                                    */
/* ========================================================================= */
static uint8_t  drive_mode;          /* 0..3 or 0xFF                          */
static uint8_t  slip_level;          /* 0..7 from ADC                         */

/* Last-sent snapshot for change detection */
static uint8_t  last_sent_drive_mode;
static uint8_t  last_sent_slip_level;

/* ADC moving-average filter ring buffer */
static uint16_t adc_ring[ADC_FILTER_DEPTH];
static uint8_t  adc_ring_idx;
static uint8_t  adc_ring_full;
static uint32_t adc_sum;

/* Button debounce state machine (per button) */
static btn_debounce_t db_pb12;
static btn_debounce_t db_pb13;
static btn_debounce_t db_s1;
static btn_debounce_t db_s2;
static btn_debounce_t db_s3;
static btn_debounce_t db_s4;

/* Timing book-keeping */
static uint32_t last_button_scan_ms;
static uint32_t last_led2_toggle_ms;
static uint32_t led1_off_until_ms;    /* LED1 restore timestamp (active-low)  */
static uint32_t drive_mode_hold_until_ms; /* Rotary muted until this tick     */

/* External references (can.c globals) */
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef  TxHeader;
extern ADC_HandleTypeDef    hadc1;

/* ========================================================================= */
/* Static helper prototypes                                                  */
/* ========================================================================= */
static void      ADC_Read_And_Filter(void);
static void      Scan_Buttons(void);
static void      Update_Drive_Mode(uint32_t now);
static void      Send_CAN_SlipLevel(void);
static void      Send_CAN_RecorderToggle(void);
static void      Send_CAN_ErrorClear(void);
static void      Send_CAN_DriveMode(void);
static void      Flash_LED1(void);
static uint8_t   Debounce_Sample(GPIO_TypeDef *port, uint16_t pin,
                                 btn_debounce_t *db);

/* ========================================================================= */
/* Public API                                                                */
/* ========================================================================= */

/**
 * @brief  Initialise the button-panel module.
 * @note   Call once after MX_ADC_Init() and CAN filter setup.
 */
void ButtonPanel_Init(void)
{
    /* State variables */
    drive_mode           = DRIVE_MODE_INVALID;
    slip_level           = 0U;

    /* Last-sent snapshot — force first TX by setting to impossible values */
    last_sent_drive_mode = 0xFFU;
    last_sent_slip_level = 0xFFU;

    /* ADC filter ring */
    for (uint8_t i = 0U; i < ADC_FILTER_DEPTH; i++) {
        adc_ring[i] = 0U;
    }
    adc_ring_idx  = 0U;
    adc_ring_full = 0U;
    adc_sum       = 0U;

    /* Debounce state — assume idle (=1 because pull-up, button not pressed) */
    db_pb12.stable = 1U;  db_pb12.level = 1U;  db_pb12.cnt = 0U;
    db_pb13.stable = 1U;  db_pb13.level = 1U;  db_pb13.cnt = 0U;
    db_s1.stable   = 1U;  db_s1.level   = 1U;  db_s1.cnt   = 0U;
    db_s2.stable   = 1U;  db_s2.level   = 1U;  db_s2.cnt   = 0U;
    db_s3.stable   = 1U;  db_s3.level   = 1U;  db_s3.cnt   = 0U;
    db_s4.stable   = 1U;  db_s4.level   = 1U;  db_s4.cnt   = 0U;

    /* Timing */
    last_button_scan_ms = HAL_GetTick();
    last_led2_toggle_ms = HAL_GetTick();
    led1_off_until_ms   = 0U;
    drive_mode_hold_until_ms = 0U;

    /* LEDs off (active-low: high = off) */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    /* Prime the ADC filter with a few initial readings */
    for (uint8_t i = 0U; i < ADC_FILTER_DEPTH; i++) {
        ADC_Read_And_Filter();
    }

    /* Immediately read initial drive mode */
    Update_Drive_Mode(HAL_GetTick());
}

/**
 * @brief  Periodic process — call from main loop as fast as possible.
 *         Sub-tasks are rate-limited internally.
 *         CAN messages are sent only when data has changed.
 */
void ButtonPanel_Process(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- LED1 flash restore: turn off after 100 ms low pulse ---- */
    if ((led1_off_until_ms > 0U) && (now >= led1_off_until_ms)) {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        led1_off_until_ms = 0U;
    }

    /* ---- LED2 1 Hz blink (toggle every 500 ms) ---- */
    if ((now - last_led2_toggle_ms) >= 500U) {
        last_led2_toggle_ms = now;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }

    /* ---- Button scan – every BUTTON_SCAN_PERIOD_MS ---- */
    if ((now - last_button_scan_ms) >= BUTTON_SCAN_PERIOD_MS) {
        last_button_scan_ms = now;
        Scan_Buttons();        /* rising-edge → recorder/error-clear TX */
        Update_Drive_Mode(now);
        ADC_Read_And_Filter();

        /* ---- Slip level change detection → 0x700 ---- */
        if (slip_level != last_sent_slip_level) {
            last_sent_slip_level = slip_level;
            Send_CAN_SlipLevel();
            Flash_LED1();
        }

        /* ---- Drive mode change detection → 0x310 ---- */
        if (drive_mode != last_sent_drive_mode) {
            last_sent_drive_mode = drive_mode;
            Send_CAN_DriveMode();
            Flash_LED1();
            /* Mute the rotary input until the knob contacts settle; bounce
               must not retrigger 0x310 and flap the reported mode. */
            drive_mode_hold_until_ms = now + DRIVE_MODE_HOLD_MS;
        }
    }
}

/* ========================================================================= */
/* Static helpers                                                            */
/* ========================================================================= */

/**
 * @brief  Software-trigger one ADC conversion, read the value, and feed the
 *         moving-average ring buffer.  The averaged result is stored in
 *         slip_level (0..7).
 */
static void ADC_Read_And_Filter(void)
{
    uint32_t filtered;
    uint16_t raw;

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return;
    }

    raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* ---- Moving-average ring buffer ---- */
    adc_sum -= adc_ring[adc_ring_idx];
    adc_ring[adc_ring_idx] = raw;
    adc_sum += raw;
    adc_ring_idx++;
    if (adc_ring_idx >= ADC_FILTER_DEPTH) {
        adc_ring_idx  = 0U;
        adc_ring_full = 1U;
    }

    uint8_t count = adc_ring_full ? ADC_FILTER_DEPTH : adc_ring_idx;
    if (count == 0U) { count = 1U; }
    filtered = adc_sum / count;

    /* Quantise: 12-bit ADC (0..4095) → 8 levels (0..7) */
    slip_level = (uint8_t)((filtered * ADC_SLIP_LEVELS) / 4096U);
}

/**
 * @brief  Scan the two push-buttons with software debounce.
 *
 *   PB12: rising edge → send recorder/camera toggle (0x701)
 *   PB13: rising edge → send error clear (0x702)
 */
static void Scan_Buttons(void)
{
    uint8_t pb12_old = db_pb12.level;
    uint8_t pb13_old = db_pb13.level;

    uint8_t pb12_new = Debounce_Sample(KEY1_GPIO_Port, KEY1_Pin, &db_pb12);
    uint8_t pb13_new = Debounce_Sample(KEY2_GPIO_Port, KEY2_Pin, &db_pb13);

    /* Rising edge: old=0 (pressed) → new=1 (released) */
    if (pb12_old == 0U && pb12_new == 1U) {
        Send_CAN_RecorderToggle();
        Flash_LED1();
    }
    if (pb13_old == 0U && pb13_new == 1U) {
        Send_CAN_ErrorClear();
        Flash_LED1();
    }
}

/**
 * @brief  Debounce a single button input.
 * @return Current stable level (0 or 1).
 */
static uint8_t Debounce_Sample(GPIO_TypeDef *port, uint16_t pin,
                               btn_debounce_t *db)
{
    uint8_t raw = (uint8_t)((port->IDR & pin) ? 1U : 0U);

    if (raw == db->level) {
        db->cnt = 0U;
    } else {
        db->cnt++;
        if (db->cnt >= BUTTON_DEBOUNCE_CNT) {
            db->level  = raw;
            db->cnt    = 0U;
            db->stable = 1U;
        }
    }
    return db->level;
}

/**
 * @brief  Read the 4-position rotary switch and update drive_mode.
 *
 *         S1=PA6, S2=PA7, S3=PB0, S4=PB1  (all active-low with pull-up)
 * @note   Skipped while inside the DRIVE_MODE_HOLD_MS window after a
 *         0x310 send, so the knob can settle without triggering a send.
 */
static void Update_Drive_Mode(uint32_t now)
{
    if ((int32_t)(now - drive_mode_hold_until_ms) < 0) {
        return;
    }

    Debounce_Sample(S1_GPIO_Port, S1_Pin, &db_s1);
    Debounce_Sample(S2_GPIO_Port, S2_Pin, &db_s2);
    Debounce_Sample(S3_GPIO_Port, S3_Pin, &db_s3);
    Debounce_Sample(S4_GPIO_Port, S4_Pin, &db_s4);

    uint8_t s1 = db_s1.level;
    uint8_t s2 = db_s2.level;
    uint8_t s3 = db_s3.level;
    uint8_t s4 = db_s4.level;

    uint8_t active_cnt = (uint8_t)((s1 == 0U) + (s2 == 0U)
                                 + (s3 == 0U) + (s4 == 0U));

    if (active_cnt == 1U) {
        if      (s1 == 0U) { drive_mode = DRIVE_MODE_S1; }
        else if (s2 == 0U) { drive_mode = DRIVE_MODE_S2; }
        else if (s3 == 0U) { drive_mode = DRIVE_MODE_S3; }
        else               { drive_mode = DRIVE_MODE_S4; }
    } else {
        drive_mode = DRIVE_MODE_INVALID;
    }
}

/**
 * @brief  Send slip level on CAN ID 0x700 (single byte, 0..7).
 */
static void Send_CAN_SlipLevel(void)
{
    uint8_t data[1] = { slip_level };

    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.StdId              = CAN_ID_SLIP_LEVEL;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC                = 1U;
    uint32_t mail;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mail);
}

/**
 * @brief  Send recorder/camera toggle on CAN ID 0x701.
 *         bit0 = 1 means "toggle recorder/camera state".
 */
static void Send_CAN_RecorderToggle(void)
{
    uint8_t data[1] = { 0x01U };

    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.StdId              = CAN_ID_RECORDER_TOGGLE;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC                = 1U;
    uint32_t mail;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mail);
}

/**
 * @brief  Send error clear on CAN ID 0x702.
 *         bit0 = 1 means "clear error".
 */
static void Send_CAN_ErrorClear(void)
{
    uint8_t data[1] = { 0x01U };

    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.StdId              = CAN_ID_ERROR_CLEAR;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC                = 1U;
    uint32_t mail;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mail);
}

/**
 * @brief  Flash LED1 with a brief low pulse (100 ms).
 */
static void Flash_LED1(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    led1_off_until_ms = HAL_GetTick() + 100U;
}

/**
 * @brief  Send drive mode on CAN ID 0x310 (vehicle DriveMode_Request).
 *         Encodes internal 0..3 as ASCII '0'..'3' (48..51).
 */
static void Send_CAN_DriveMode(void)
{
    uint8_t data[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    data[0] = (drive_mode <= DRIVE_MODE_S4)
                ? (uint8_t)(drive_mode + DRIVE_MODE_ASCII_OFFSET)
                : DRIVE_MODE_INVALID;

    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.StdId              = CAN_ID_DRIVE_MODE;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.DLC                = 8U;
    uint32_t mail;
    HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mail);
}
