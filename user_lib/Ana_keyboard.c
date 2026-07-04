#include "Ana_keyboard.h"

typedef struct
{
  AnaKeyboard_Key_t key;
  uint16_t adc10_center;
  char key_char;
} AnaKeyboard_Map_t;

static const AnaKeyboard_Map_t s_key_map[] =
{
  {ANA_KEY_1,    36U, '1'},
  {ANA_KEY_2,    76U, '2'},
  {ANA_KEY_3,   108U, '3'},
  {ANA_KEY_4,   139U, '4'},
  {ANA_KEY_5,   169U, '5'},
  {ANA_KEY_6,   196U, '6'},
  {ANA_KEY_7,   216U, '7'},
  {ANA_KEY_8,   235U, '8'},
  {ANA_KEY_9,   252U, '9'},
  {ANA_KEY_STAR,270U, '*'},
  {ANA_KEY_0,   285U, '0'},
  {ANA_KEY_HASH,307U, '#'}
};

static ADC_HandleTypeDef *s_hadc = NULL;
static AnaKeyboard_Key_t s_current_key = ANA_KEY_NONE;
static AnaKeyboard_Key_t s_candidate_key = ANA_KEY_NONE;
static uint8_t s_debounce_count = 0U;
static uint8_t s_wait_release = 0U;

volatile uint32_t AnaKeyboard_RawAdcValue = 0U;
volatile uint16_t AnaKeyboard_Adc10Value = 1023U;
volatile char     AnaKeyboard_CurrentKeyChar = 0;
volatile char     AnaKeyboard_LastEventChar = 0;

static uint32_t AnaKeyboard_GetAdcFullScale(void)
{
  if (s_hadc == NULL)
  {
    return 65535U;
  }

  switch (s_hadc->Init.Resolution)
  {
#if defined(ADC_RESOLUTION_16B)
    case ADC_RESOLUTION_16B:
      return 65535U;
#endif
#if defined(ADC_RESOLUTION_14B)
    case ADC_RESOLUTION_14B:
      return 16383U;
#endif
#if defined(ADC_RESOLUTION_12B)
    case ADC_RESOLUTION_12B:
      return 4095U;
#endif
#if defined(ADC_RESOLUTION_10B)
    case ADC_RESOLUTION_10B:
      return 1023U;
#endif
#if defined(ADC_RESOLUTION_8B)
    case ADC_RESOLUTION_8B:
      return 255U;
#endif
    default:
      return 65535U;
  }
}

static uint16_t AnaKeyboard_RawToAdc10(uint32_t raw)
{
  uint32_t full_scale = AnaKeyboard_GetAdcFullScale();

  if (raw > full_scale)
  {
    raw = full_scale;
  }

  return (uint16_t)((raw * 1023U + (full_scale / 2U)) / full_scale);
}

static AnaKeyboard_Key_t AnaKeyboard_DecodeAdc10(uint16_t adc10)
{
  uint32_t i;
  uint16_t best_diff = 0xFFFFU;
  AnaKeyboard_Key_t best_key = ANA_KEY_NONE;

  if (adc10 >= ANA_KEY_RELEASE_ADC10_MIN)
  {
    return ANA_KEY_NONE;
  }

  for (i = 0U; i < (sizeof(s_key_map) / sizeof(s_key_map[0])); i++)
  {
    uint16_t center = s_key_map[i].adc10_center;
    uint16_t diff = (adc10 > center) ? (uint16_t)(adc10 - center) : (uint16_t)(center - adc10);

    if (diff < best_diff)
    {
      best_diff = diff;
      best_key = s_key_map[i].key;
    }
  }

  if (best_diff <= ANA_KEY_ADC10_TOLERANCE)
  {
    return best_key;
  }

  return ANA_KEY_NONE;
}

void AnaKeyboard_Init(ADC_HandleTypeDef *hadc)
{
  s_hadc = hadc;
  s_current_key = ANA_KEY_NONE;
  s_candidate_key = ANA_KEY_NONE;
  s_debounce_count = 0U;
  s_wait_release = 0U;

  AnaKeyboard_RawAdcValue = 0U;
  AnaKeyboard_Adc10Value = 1023U;
  AnaKeyboard_CurrentKeyChar = 0;
  AnaKeyboard_LastEventChar = 0;

#if defined(ADC_CALIB_OFFSET) && defined(ADC_SINGLE_ENDED)
  /* ADC calibration is recommended on STM32H7 before the first conversion. */
  if (s_hadc != NULL)
  {
    (void)HAL_ADCEx_Calibration_Start(s_hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  }
#endif
}

uint32_t AnaKeyboard_ReadRaw(void)
{
  uint32_t sum = 0U;
  uint32_t i;
  uint32_t samples = ANA_KEY_AVERAGE_SAMPLES;

  if (s_hadc == NULL)
  {
    return 0U;
  }

  if (samples == 0U)
  {
    samples = 1U;
  }

  for (i = 0U; i < samples; i++)
  {
    if (HAL_ADC_Start(s_hadc) != HAL_OK)
    {
      break;
    }

    if (HAL_ADC_PollForConversion(s_hadc, 10U) == HAL_OK)
    {
      sum += HAL_ADC_GetValue(s_hadc);
    }

    (void)HAL_ADC_Stop(s_hadc);
  }

  if (i == 0U)
  {
    return 0U;
  }

  AnaKeyboard_RawAdcValue = (sum + (i / 2U)) / i;
  return AnaKeyboard_RawAdcValue;
}

uint16_t AnaKeyboard_ReadAdc10(void)
{
  AnaKeyboard_Adc10Value = AnaKeyboard_RawToAdc10(AnaKeyboard_ReadRaw());
  return AnaKeyboard_Adc10Value;
}

AnaKeyboard_Key_t AnaKeyboard_Update(void)
{
  AnaKeyboard_Key_t decoded_key = AnaKeyboard_DecodeAdc10(AnaKeyboard_ReadAdc10());
  AnaKeyboard_Key_t event_key = ANA_KEY_NONE;

  if (decoded_key == ANA_KEY_NONE)
  {
    s_current_key = ANA_KEY_NONE;
    s_candidate_key = ANA_KEY_NONE;
    s_debounce_count = 0U;
    s_wait_release = 0U;
    AnaKeyboard_CurrentKeyChar = 0;
    return ANA_KEY_NONE;
  }

  if (decoded_key == s_candidate_key)
  {
    if (s_debounce_count < 255U)
    {
      s_debounce_count++;
    }
  }
  else
  {
    s_candidate_key = decoded_key;
    s_debounce_count = 1U;
  }

  if ((s_debounce_count >= ANA_KEY_DEBOUNCE_COUNT) && (s_wait_release == 0U))
  {
    s_current_key = decoded_key;
    AnaKeyboard_CurrentKeyChar = AnaKeyboard_KeyToChar(decoded_key);
    AnaKeyboard_LastEventChar = AnaKeyboard_CurrentKeyChar;
    s_wait_release = 1U;
    event_key = decoded_key;
    AnaKeyboard_KeyDownCallback(decoded_key);
  }

  return event_key;
}

AnaKeyboard_Key_t AnaKeyboard_GetCurrentKey(void)
{
  return s_current_key;
}

char AnaKeyboard_KeyToChar(AnaKeyboard_Key_t key)
{
  uint32_t i;

  for (i = 0U; i < (sizeof(s_key_map) / sizeof(s_key_map[0])); i++)
  {
    if (s_key_map[i].key == key)
    {
      return s_key_map[i].key_char;
    }
  }

  return 0;
}

__weak void AnaKeyboard_KeyDownCallback(AnaKeyboard_Key_t key)
{
  (void)key;
}
