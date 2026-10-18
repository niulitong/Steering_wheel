/**
 * @file    button_panel.h
 * @brief   Steering wheel button panel module header.
 *          Handles:
 *            - PB12: recorder/camera toggle (rising edge → TX on 0x701)
 *            - PB13: error-clear trigger (rising edge → TX on 0x702)
 *            - PA6/PA7/PB0/PB1: 4-position rotary switch → drive mode
 *            - PA1 ADC: 0-3.3V → 8-level slip rate with moving-average filter
 *          Slip level is sent on CAN ID 0x700 on change.
 *          Drive mode is sent on CAN ID 0x310 on change; the rotary pins
 *          are then not sampled for DRIVE_MODE_HOLD_MS so contact bounce
 *          while the knob settles cannot flap the reported mode.
 */
#ifndef __BUTTON_PANEL_H__
#define __BUTTON_PANEL_H__

#include "main.h"

/* ------------------------------------------------------------------------- */
/* CAN identifiers                                                           */
/* ------------------------------------------------------------------------- */
#define CAN_ID_SLIP_LEVEL       0x700U   /* Slip level (0..7)                 */
#define CAN_ID_RECORDER_TOGGLE  0x701U   /* Recorder/camera toggle (bit0)     */
#define CAN_ID_ERROR_CLEAR      0x702U   /* Error clear (bit0)                */
#define CAN_ID_DRIVE_MODE       0x310U   /* DriveMode_Request (vehicle protocol) */

/* ------------------------------------------------------------------------- */
/* Drive mode codes (rotary-switch positions)                                */
/*   Internal 0..3 is offset by DRIVE_MODE_ASCII_OFFSET when sent on 0x310   */
/*   to yield ASCII '0'..'3' (48..51).                                       */
/* ------------------------------------------------------------------------- */
#define DRIVE_MODE_S1           0x00U
#define DRIVE_MODE_S2           0x01U
#define DRIVE_MODE_S3           0x02U
#define DRIVE_MODE_S4           0x03U
#define DRIVE_MODE_INVALID      0xFFU

/* Offset applied when sending drive mode on 0x310 (ASCII '0'..'3' = 48..51) */
#define DRIVE_MODE_ASCII_OFFSET 0x30U

/* ------------------------------------------------------------------------- */
/* ADC filter parameters                                                     */
/* ------------------------------------------------------------------------- */
#define ADC_FILTER_DEPTH        16U     /* Moving-average window size        */
#define ADC_SLIP_LEVELS         8U      /* Number of slip-rate steps         */

/* ------------------------------------------------------------------------- */
/* Button scan / debounce parameters                                         */
/* ------------------------------------------------------------------------- */
#define BUTTON_SCAN_PERIOD_MS   10U     /* Scan interval in ms               */
#define BUTTON_DEBOUNCE_CNT      4U     /* Consecutive stable reads needed   */

/* ------------------------------------------------------------------------- */
/* Drive mode hold (rotary switch)                                           */
/*   After 0x310 is sent, S1..S4 are not sampled for this long. Contact      */
/*   bounce while the knob settles is thereby ignored.                       */
/* ------------------------------------------------------------------------- */
#define DRIVE_MODE_HOLD_MS      200U    /* Post-send rotary mute in ms       */

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
void ButtonPanel_Init(void);
void ButtonPanel_Process(void);

#endif /* __BUTTON_PANEL_H__ */
