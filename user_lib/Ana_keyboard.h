#ifndef __ANA_KEYBOARD_H
#define __ANA_KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"

/*
 * 12-key analog matrix keyboard driver.
 * Hardware connection: keyboard S -> ADC1_INP11(PC1), V -> 3.3V, G -> GND.
 *
 * The key values below are based on the Arduino 10-bit reference values:
 * 1:36, 2:76, 3:108, 4:139, 5:169, 6:196,
 * 7:216, 8:235, 9:252, *:270, 0:285, #:307, released:1023.
 *
 * This driver converts the current STM32 ADC resolution value to an equivalent
 * 10-bit value, so it can work with your current CubeMX ADC1 16-bit setting.
 */

#ifndef ANA_KEY_ADC10_TOLERANCE
#define ANA_KEY_ADC10_TOLERANCE       8U
#endif

#ifndef ANA_KEY_RELEASE_ADC10_MIN
#define ANA_KEY_RELEASE_ADC10_MIN     1000U
#endif

#ifndef ANA_KEY_AVERAGE_SAMPLES
#define ANA_KEY_AVERAGE_SAMPLES       8U
#endif

#ifndef ANA_KEY_DEBOUNCE_COUNT
#define ANA_KEY_DEBOUNCE_COUNT        3U
#endif

typedef enum
{
  ANA_KEY_NONE = 0,
  ANA_KEY_1,
  ANA_KEY_2,
  ANA_KEY_3,
  ANA_KEY_4,
  ANA_KEY_5,
  ANA_KEY_6,
  ANA_KEY_7,
  ANA_KEY_8,
  ANA_KEY_9,
  ANA_KEY_STAR,
  ANA_KEY_0,
  ANA_KEY_HASH
} AnaKeyboard_Key_t;

/* Variables for live watch/debug in Keil. */
extern volatile uint32_t AnaKeyboard_RawAdcValue;
extern volatile uint16_t AnaKeyboard_Adc10Value;
extern volatile char     AnaKeyboard_CurrentKeyChar;
extern volatile char     AnaKeyboard_LastEventChar;

void AnaKeyboard_Init(ADC_HandleTypeDef *hadc);
AnaKeyboard_Key_t AnaKeyboard_Update(void);
AnaKeyboard_Key_t AnaKeyboard_GetCurrentKey(void);
uint32_t AnaKeyboard_ReadRaw(void);
uint16_t AnaKeyboard_ReadAdc10(void);
char AnaKeyboard_KeyToChar(AnaKeyboard_Key_t key);

/* Override this function in main.c if you want to handle key-down events there. */
void AnaKeyboard_KeyDownCallback(AnaKeyboard_Key_t key);

#ifdef __cplusplus
}
#endif

#endif /* __ANA_KEYBOARD_H */
