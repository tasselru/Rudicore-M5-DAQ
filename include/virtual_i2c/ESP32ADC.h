#pragma once

#include "esp32-hal-adc.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

/* --- ESP32 ADC (virtual I2C addr D2) ---------------------------------------
 * Command strings:
 *  >ESP32ADC.Configure([Port=35|36|Both, FirstChannel=35|36|2|5, SecondChannel=35|36|2|5, Attenuation=0dB|2.5dB|6dB|11dB, MarkChannelTwo=true|false, MeasuringInterval=, SamplingRate=, SampleSize=, TriggerEvent=IGNORE|LOW|HIGH|CHANGE|RISING|FALLING, Format=Byte|String])
 *  >ESP32ADC.GetConfiguration([BufferSize])
 *  >ESP32ADC.GetValue([Port=35|36|Both])
 *  >ESP32ADC.GetEnvelope([Duration=1000, Port=35|36|Both])
 *  >ESP32ADC.AcquireToBuffer()
 *  >ESP32ADC.AcquireToBufferMaximumSpeed()
 *  >ESP32ADC.GetBuffer([Format=Byte|String])
 *  >ESP32ADC.GetAvailableSampleCount()
 *  >ESP32ADC.GetStream()
 *  >ESP32ADC.SetAttenuation(Factor=0dB|2.5dB|6dB|11dB)
 *  >ESP32ADC.Initialize()
 *
 * ConfigureBufferedRead remains supported as a compatibility alias for Configure.
 */

#define ESP_ADC1Port 35
#define ESP_ADC2Port 36
#define ESP_TriggerPort 5

static bool ESP_ADC1 = true;
static bool ESP_ADC2 = false;
static bool ESP_Both = false;
static bool ESP_MarkChannelTwo = false;
static int ESP_FirstChannel = ESP_ADC1Port;
static int ESP_SecondChannel = ESP_ADC2Port;

#define ESP_MeasuringIntervalDefault 1000
static unsigned long ESP_MeasuringInterval = ESP_MeasuringIntervalDefault;
#define ESP_MeasuringIntervalMax 100000

#define ESP_SamplingRateDefault 1000
static unsigned long ESP_SamplingRate = ESP_SamplingRateDefault;
#define ESP_SamplingRateMax 100000

/* Compile-time oversampling factor used for analog reads in dual-channel
 * buffered acquisition, streaming/envelope average helpers, and GetValue(Both).
 * Change this #define and rebuild to adjust oversampling.
 */
#ifndef ESP_ADC_OVERSAMPLING
#define ESP_ADC_OVERSAMPLING 16
#endif

#if ESP_ADC_OVERSAMPLING < 1
#error "ESP_ADC_OVERSAMPLING must be at least 1"
#endif

#if ESP_ADC_OVERSAMPLING > 64
#error "ESP_ADC_OVERSAMPLING must be 64 or less"
#endif

#define ESP_SampleSizeDefault 1000
static unsigned long ESP_SampleSize = ESP_SampleSizeDefault;
#define ESP_SampleSizeMax data_buf_size

#define ESP_AttenuationDefault "11DB"
static String ESP_Attenuation = ESP_AttenuationDefault;

static const String ESP_TriggerEvents[] = {"IGNORE", "LOW", "HIGH", "CHANGE", "RISING", "FALLING"};
#define Ignore 0
#define Low 1
#define High 2
#define Change 3
#define Rising 4
#define Falling 5
static int ESP_DefaultTriggerEvent = Ignore;
static int ESP_TriggerEvent = ESP_DefaultTriggerEvent;
#define ESP_TriggerTimeout 20000

static bool ESP_Byte = false;
static volatile unsigned long ESP_AvailableSampleCount = 0;

/* Fast local ADC path based on the Arduino ESP32 1.0.4 esp32-hal-adc.c
 * implementation. It is intentionally not exported as analogRead(), so only
 * the ESP32ADC virtual device bypasses the slower PlatformIO core call.
 */
static uint8_t ESP32_ADC_FastAttenuation = 3;
static uint8_t ESP32_ADC_FastWidth = 3;
static uint8_t ESP32_ADC_FastCycles = 8;
static uint8_t ESP32_ADC_FastSamples = 0;
static uint8_t ESP32_ADC_FastClockDiv = 1;
static uint8_t ESP32_ADC_FastReturnedWidth = 12;

static void ESP32_ADC_FastSetWidth(uint8_t bits)
{
  if (bits < 9) bits = 9;
  else if (bits > 12) bits = 12;

  ESP32_ADC_FastReturnedWidth = bits;
  ESP32_ADC_FastWidth = bits - 9;
  SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR1_BIT_WIDTH, ESP32_ADC_FastWidth, SENS_SAR1_BIT_WIDTH_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_BIT, ESP32_ADC_FastWidth, SENS_SAR1_SAMPLE_BIT_S);
  SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR2_BIT_WIDTH, ESP32_ADC_FastWidth, SENS_SAR2_BIT_WIDTH_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_BIT, ESP32_ADC_FastWidth, SENS_SAR2_SAMPLE_BIT_S);
}

static void ESP32_ADC_FastSetCycles(uint8_t cycles)
{
  ESP32_ADC_FastCycles = cycles;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, ESP32_ADC_FastCycles, SENS_SAR1_SAMPLE_CYCLE_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_CYCLE, ESP32_ADC_FastCycles, SENS_SAR2_SAMPLE_CYCLE_S);
}

static void ESP32_ADC_FastSetSamples(uint8_t samples)
{
  if (!samples) return;
  ESP32_ADC_FastSamples = samples - 1;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_NUM, ESP32_ADC_FastSamples, SENS_SAR1_SAMPLE_NUM_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_NUM, ESP32_ADC_FastSamples, SENS_SAR2_SAMPLE_NUM_S);
}

static void ESP32_ADC_FastSetClockDiv(uint8_t clockDiv)
{
  if (!clockDiv) return;
  ESP32_ADC_FastClockDiv = clockDiv;
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_CLK_DIV, ESP32_ADC_FastClockDiv, SENS_SAR1_CLK_DIV_S);
  SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_CLK_DIV, ESP32_ADC_FastClockDiv, SENS_SAR2_CLK_DIV_S);
}

static void ESP32_ADC_FastSetAttenuation(adc_attenuation_t attenuation)
{
  ESP32_ADC_FastAttenuation = attenuation & 3;
  uint32_t att_data = 0;
  int i = 10;
  while (i--) att_data |= ESP32_ADC_FastAttenuation << (i * 2);
  WRITE_PERI_REG(SENS_SAR_ATTEN1_REG, att_data & 0xFFFF);
  WRITE_PERI_REG(SENS_SAR_ATTEN2_REG, att_data);
}

static void IRAM_ATTR ESP32_ADC_FastInit()
{
  static bool initialized = false;
  if (initialized) return;

  ESP32_ADC_FastSetAttenuation((adc_attenuation_t)ESP32_ADC_FastAttenuation);
  ESP32_ADC_FastSetCycles(ESP32_ADC_FastCycles);
  ESP32_ADC_FastSetSamples(ESP32_ADC_FastSamples + 1);
  ESP32_ADC_FastSetClockDiv(ESP32_ADC_FastClockDiv);
  ESP32_ADC_FastSetWidth(ESP32_ADC_FastWidth + 9);

  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
  SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_M);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_M);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE_M);
  SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE_M);
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_M);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S);
  CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
  while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0) {};

  initialized = true;
}

static void ESP32_ADC_FastSetPinAttenuation(uint8_t pin, adc_attenuation_t attenuation)
{
  int8_t channel = digitalPinToAnalogChannel(pin);
  if ((channel < 0) || (attenuation > 3)) return;
  ESP32_ADC_FastInit();
  if (channel > 7) SET_PERI_REG_BITS(SENS_SAR_ATTEN2_REG, 3, attenuation, ((channel - 10) * 2));
  else SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, 3, attenuation, (channel * 2));
}

static bool IRAM_ATTR ESP32_ADC_FastAttachPin(uint8_t pin)
{
  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) return false;

  int8_t pad = digitalPinToTouchChannel(pin);
  if (pad >= 0)
  {
    uint32_t touch = READ_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG);
    if (touch & (1 << pad))
    {
      touch &= ~((1 << (pad + SENS_TOUCH_PAD_OUTEN2_S)) |
                 (1 << (pad + SENS_TOUCH_PAD_OUTEN1_S)) |
                 (1 << (pad + SENS_TOUCH_PAD_WORKEN_S)));
      WRITE_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG, touch);
    }
  }
  else if (pin == 25)
  {
    CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);
  }
  else if (pin == 26)
  {
    CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);
  }

  pinMode(pin, ANALOG);
  ESP32_ADC_FastInit();
  return true;
}

static bool IRAM_ATTR ESP32_ADC_FastStart(uint8_t pin)
{
  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) return false;

  if (channel > 9)
  {
    channel -= 10;
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
  }
  else
  {
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  }
  return true;
}

static uint16_t IRAM_ATTR ESP32_ADC_FastEnd(uint8_t pin)
{
  uint16_t value = 0;
  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) return 0;

  if (channel > 7)
  {
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0) {};
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
  }
  else
  {
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0) {};
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
  }

  uint8_t from = ESP32_ADC_FastWidth + 9;
  if (from == ESP32_ADC_FastReturnedWidth) return value;
  if (from > ESP32_ADC_FastReturnedWidth) return value >> (from - ESP32_ADC_FastReturnedWidth);
  return value << (ESP32_ADC_FastReturnedWidth - from);
}

static uint16_t IRAM_ATTR ESP32_ADC_FastAnalogRead(uint8_t pin)
{
  if (!ESP32_ADC_FastAttachPin(pin) || !ESP32_ADC_FastStart(pin)) return 0;
  return ESP32_ADC_FastEnd(pin);
}

/* Use this inside tight acquisition loops after the analog pins have already
 * been attached/prepared by ESP32_ADC_init(). This avoids pinMode()/attach
 * work on every sample, which can change timing and settling behavior.
 */
static uint16_t IRAM_ATTR ESP32_ADC_FastAnalogReadPrepared(uint8_t pin)
{
  if (!ESP32_ADC_FastStart(pin)) return 0;
  return ESP32_ADC_FastEnd(pin);
}

static void ESP32_ADC_FastReadResolution(uint8_t bits)
{
  if (!bits || bits > 16) return;
  ESP32_ADC_FastSetWidth(bits);
  ESP32_ADC_FastReturnedWidth = bits;
}

static bool ESP32_ADC_TriggerTimedOut(uint32_t start_time)
{
  if ((uint32_t)(millis() - start_time) < ESP_TriggerTimeout) return false;
  LastError("Trigger timeout");
  return true;
}

static bool ESP32_ADC_WaitWhile(int tp, int state, uint32_t start_time)
{
  while (digitalRead(tp) == state)
  {
    if (ESP32_ADC_TriggerTimedOut(start_time)) return false;
  }
  return true;
}

static bool ESP32_ADC_Trigger(int tp, int event)
{
  pinMode(tp, INPUT_PULLUP);
  uint32_t start_time = millis();

  switch (event)
  {
    case Ignore:  return true;
    case Low:     return ESP32_ADC_WaitWhile(tp, HIGH, start_time);
    case High:    return ESP32_ADC_WaitWhile(tp, LOW, start_time);
    case Change:
    {
      bool initial = digitalRead(tp);
      while (digitalRead(tp) == initial)
      {
        if (ESP32_ADC_TriggerTimedOut(start_time)) return false;
      }
      return true;
    }
    case Rising:
      if (!ESP32_ADC_WaitWhile(tp, HIGH, start_time)) return false;
      return ESP32_ADC_WaitWhile(tp, LOW, start_time);
    case Falling:
      if (!ESP32_ADC_WaitWhile(tp, LOW, start_time)) return false;
      return ESP32_ADC_WaitWhile(tp, HIGH, start_time);
    default:
      return false;
  }
}

static bool ESP32_ADC_ChannelValid(int channel)
{
  return ((channel == ESP_ADC1Port) || (channel == ESP_ADC2Port) || (channel == 2) || (channel == ESP_TriggerPort));
}

static bool ESP32_ADC_ChannelAnalog(int channel)
{
  return ((channel == ESP_ADC1Port) || (channel == ESP_ADC2Port));
}

static uint16_t IRAM_ATTR ESP32_ADC_Read_Analog_Average(uint8_t channel, uint16_t oversampling_factor)
{
  uint32_t value = 0;
  for (uint16_t avg_i = 0; avg_i < oversampling_factor; avg_i++)
  {
    value += ESP32_ADC_FastAnalogReadPrepared(channel);
  }
  return value / oversampling_factor;
}

static uint16_t ESP32_ADC_Read_Channel(int channel)
{
  if (ESP32_ADC_ChannelAnalog(channel))
  {
    return ESP32_ADC_Read_Analog_Average(channel, ESP_ADC_OVERSAMPLING);
  }
  return digitalRead(channel) * 4095;
}

#define ESP32_ADC_READ_ANALOG_FAST(channel) ESP32_ADC_FastAnalogReadPrepared(channel)
#define ESP32_ADC_READ_ANALOG_AVERAGE(channel) ESP32_ADC_Read_Analog_Average(channel, ESP_ADC_OVERSAMPLING)
#define ESP32_ADC_READ_DIGITAL(channel) (digitalRead(channel) * 4095)

#define ESP32_ADC_STREAM_BOTH(first_read, second_read) \
  do \
  { \
    do \
    { \
      time_target = micros(); \
      ADC_val = (first_read); \
      ADC_val_2 = (second_read); \
      if (ESP_MarkChannelTwo) bitSet(ADC_val_2, 12); \
      if (ESP_Byte) \
      { \
        write2serial((ADC_val >> 8) & 0xff); \
        write2serial(ADC_val & 0xff); \
        write2serial((ADC_val_2 >> 8) & 0xff); \
        write2serial(ADC_val_2 & 0xff); \
      } \
      else \
      { \
        print2serial(String(ADC_val)); \
        print2serial(String(ADC_val_2)); \
      } \
      while ((uint32_t)(micros() - time_target) < sample_period_micro) {}; \
    } while (!input_available()); \
  } while (false)

#define ESP32_ADC_ENVELOPE_BOTH(first_read, second_read) \
  do \
  { \
    do \
    { \
      Ch1Val = (first_read); \
      Ch2Val = (second_read); \
      Ch1Max = max(Ch1Max, Ch1Val); \
      Ch1Min = min(Ch1Min, Ch1Val); \
      Ch2Max = max(Ch2Max, Ch2Val); \
      Ch2Min = min(Ch2Min, Ch2Val); \
    } while ((uint32_t)(millis() - start_time) < ttm); \
  } while (false)

static void ESP32_GetStream()
{
  int ADC_GPIO = ESP_ADC1Port;
  if (ESP_ADC2) ADC_GPIO = ESP_ADC2Port;

  unsigned long time_target;
  uint32_t sample_period_micro = 1000000 / ESP_SamplingRate;
  uint16_t ADC_val, ADC_val_2;
  if (!ESP32_ADC_Trigger(ESP_TriggerPort, ESP_TriggerEvent)) return;

  if (ESP_Both)
  {
    int both_channel_key = ESP_FirstChannel * 100 + ESP_SecondChannel;
    switch (both_channel_key)
    {
      case 3535: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 3536: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 3502: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_DIGITAL(2)); break;
      case 3505: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 3635: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 3636: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 3602: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_DIGITAL(2)); break;
      case 3605: ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 235:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 236:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 202:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_DIGITAL(2)); break;
      case 205:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 535:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 536:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 502:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_DIGITAL(2)); break;
      case 505:  ESP32_ADC_STREAM_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      default:
        LastError("Invalid ADC channel configuration");
        return;
    }
    return;
  }

  do
  {
    time_target = micros();
    ADC_val = ESP32_ADC_FastAnalogReadPrepared(ADC_GPIO);
    if (ESP_Byte)
    {
      write2serial((ADC_val >> 8) & 0xff);
      write2serial(ADC_val & 0xff);
    }
    else
    {
      print2serial(String(ADC_val));
    }
    while ((uint32_t)(micros() - time_target) < sample_period_micro) {};
  } while (!input_available());
}

static void ESP32_GetBuffer(bool as_bytes)
{
  if ((data_buf == nullptr) || (data_buf_size == 0))
  {
    LastError("ADC data buffer not available");
    return;
  }

  unsigned long samples = ESP_AvailableSampleCount;
  if (samples == 0) samples = data_buf_pointer_write;
  if (samples > data_buf_pointer_write) samples = data_buf_pointer_write;
  if (samples > data_buf_size) samples = data_buf_size;

  if (as_bytes)
  {
    uint8_t tx_buffer[128];
    size_t tx_count = 0;
    for (unsigned long i = 0; i < samples; i++)
    {
      tx_buffer[tx_count++] = (data_buf[i] >> 8) & 0xff;
      tx_buffer[tx_count++] = data_buf[i] & 0xff;
      if (tx_count == sizeof(tx_buffer))
      {
        write2serial(tx_buffer, tx_count);
        tx_count = 0;
      }
    }
    if (tx_count > 0) write2serial(tx_buffer, tx_count);
    return;
  }

  for (unsigned long i = 0; i < samples; i++)
  {
    print2serial(String(data_buf[i]));
  }
  print2serial("EOF");
}

static String ESP32_ADC_GetEnvelope()
{
  String ReturnValue;
  int ADC_GPIO = ESP_ADC1Port;
  if (ESP_ADC2) ADC_GPIO = ESP_ADC2Port;

  unsigned long ttm = ESP_MeasuringInterval;
  if (!ESP32_ADC_Trigger(ESP_TriggerPort, ESP_TriggerEvent)) return "-";

  if (ESP_Both)
  {
    uint16_t Ch1Max = 0;
    uint16_t Ch1Min = 4095;
    uint16_t Ch1Val = 0;
    uint16_t Ch2Max = 0;
    uint16_t Ch2Min = 4095;
    uint16_t Ch2Val = 0;

    uint32_t start_time = millis();
    int both_channel_key = ESP_FirstChannel * 100 + ESP_SecondChannel;
    switch (both_channel_key)
    {
      case 3535: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 3536: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 3502: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_DIGITAL(2)); break;
      case 3505: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 3635: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 3636: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 3602: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_DIGITAL(2)); break;
      case 3605: ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 235:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 236:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 202:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_DIGITAL(2)); break;
      case 205:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(2), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      case 535:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC1Port)); break;
      case 536:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_ANALOG_AVERAGE(ESP_ADC2Port)); break;
      case 502:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_DIGITAL(2)); break;
      case 505:  ESP32_ADC_ENVELOPE_BOTH(ESP32_ADC_READ_DIGITAL(ESP_TriggerPort), ESP32_ADC_READ_DIGITAL(ESP_TriggerPort)); break;
      default:
        LastError("Invalid ADC channel configuration");
        return "-";
    }
    ReturnValue = String(Ch1Min) + "," + String(Ch1Max) + "," + String(Ch2Min) + "," + String(Ch2Max);
    return ReturnValue;
  }

  pinMode(ADC_GPIO, INPUT);
  uint16_t PrevMax = 0;
  uint16_t PrevMin = 4095;
  uint16_t Val = 0;
  uint32_t start_time = millis();
  do
  {
    Val = ESP32_ADC_FastAnalogReadPrepared(ADC_GPIO);
    PrevMax = max(PrevMax, Val);
    PrevMin = min(PrevMin, Val);
  } while ((uint32_t)(millis() - start_time) < ttm);
  ReturnValue = String(PrevMin) + "," + String(PrevMax);
  return ReturnValue;
}

static void ESP32_ADC_Read_Analog_To_Buffer(uint8_t channel,
                                             unsigned long samples,
                                             uint32_t sample_period_micro,
                                             bool fastest)
{
  unsigned long time_target;
  uint32_t sub_sample_period_micro = sample_period_micro / ESP_ADC_OVERSAMPLING;
  if (sub_sample_period_micro == 0) sub_sample_period_micro = 1;

  do
  {
    time_target = micros();
    uint32_t analog_sum = 0;

    for (uint16_t avg_i = 0; avg_i < ESP_ADC_OVERSAMPLING; avg_i++)
    {
      unsigned long sub_time_target = micros();
      analog_sum += ESP32_ADC_FastAnalogReadPrepared(channel);

      if (!fastest && ((avg_i + 1) < ESP_ADC_OVERSAMPLING))
      {
        while ((uint32_t)(micros() - sub_time_target) < sub_sample_period_micro) {};
      }
    }

    data_buf[data_buf_pointer_write++] = analog_sum / ESP_ADC_OVERSAMPLING;

    if (!fastest) while ((uint32_t)(micros() - time_target) < sample_period_micro) {};
  } while (data_buf_pointer_write < samples);
}

static void ESP32_ADC_Read_Both_To_Buffer(uint16_t (*first_read)(), uint16_t (*second_read)(), unsigned long samples, uint32_t sample_period_micro, bool fastest)
{
  unsigned long time_target;
  do
  {
    time_target = micros();
    data_buf[data_buf_pointer_write++] = first_read();
    data_buf[data_buf_pointer_write++] = second_read();
    if (!fastest) while ((uint32_t)(micros() - time_target) < sample_period_micro) {};
  } while (data_buf_pointer_write < samples);
}

static uint16_t IRAM_ATTR ESP32_ADC_Read_Digital_2()
{
  return ESP32_ADC_READ_DIGITAL(2);
}

static uint16_t IRAM_ATTR ESP32_ADC_Read_Digital_Trigger()
{
  return ESP32_ADC_READ_DIGITAL(ESP_TriggerPort);
}

/* Dual-channel buffer acquisition helpers.
 * These are used only by AcquireToBuffer()/AcquireToBufferMaximumSpeed() when
 * Port=Both. The output rate remains ESP_SamplingRate: only one pair is written
 * to the buffer per sample period.
 *
 * For every selected analog channel, ESP_ADC_OVERSAMPLING ADC conversions are
 * read and averaged before one output sample is written. This also applies to
 * mixed analog/digital pairs: the analog channel is averaged, while the digital
 * channel is sampled during the same oversampling window and the most recent
 * digital state is written with the averaged analog value.
 */
static void ESP32_ADC_Read_Analog_Digital_To_Buffer(uint8_t analog_channel,
                                                    uint8_t digital_channel,
                                                    bool analog_first,
                                                    unsigned long samples,
                                                    uint32_t sample_period_micro,
                                                    bool fastest)
{
  unsigned long time_target;
  uint32_t sub_sample_period_micro = sample_period_micro / ESP_ADC_OVERSAMPLING;
  if (sub_sample_period_micro == 0) sub_sample_period_micro = 1;

  do
  {
    time_target = micros();
    uint32_t analog_sum = 0;
    uint16_t digital_val = 0;

    for (uint16_t avg_i = 0; avg_i < ESP_ADC_OVERSAMPLING; avg_i++)
    {
      unsigned long sub_time_target = micros();
      analog_sum += ESP32_ADC_FastAnalogReadPrepared(analog_channel);
      digital_val = ESP32_ADC_READ_DIGITAL(digital_channel);

      if (!fastest && ((avg_i + 1) < ESP_ADC_OVERSAMPLING))
      {
        while ((uint32_t)(micros() - sub_time_target) < sub_sample_period_micro) {};
      }
    }

    uint16_t analog_avg = analog_sum / ESP_ADC_OVERSAMPLING;
    if (analog_first)
    {
      data_buf[data_buf_pointer_write++] = analog_avg;
      data_buf[data_buf_pointer_write++] = digital_val;
    }
    else
    {
      data_buf[data_buf_pointer_write++] = digital_val;
      data_buf[data_buf_pointer_write++] = analog_avg;
    }

    if (!fastest) while ((uint32_t)(micros() - time_target) < sample_period_micro) {};
  } while (data_buf_pointer_write < samples);
}

static void ESP32_ADC_Read_Analog_Analog_To_Buffer(uint8_t first_channel,
                                                   uint8_t second_channel,
                                                   unsigned long samples,
                                                   uint32_t sample_period_micro,
                                                   bool fastest)
{
  unsigned long time_target;
  uint32_t sub_sample_period_micro = sample_period_micro / ESP_ADC_OVERSAMPLING;
  if (sub_sample_period_micro == 0) sub_sample_period_micro = 1;

  do
  {
    time_target = micros();
    uint32_t ch1_sum = 0;
    uint32_t ch2_sum = 0;

    for (uint16_t avg_i = 0; avg_i < ESP_ADC_OVERSAMPLING; avg_i++)
    {
      unsigned long sub_time_target = micros();
      ch1_sum += ESP32_ADC_FastAnalogReadPrepared(first_channel);
      ch2_sum += ESP32_ADC_FastAnalogReadPrepared(second_channel);

      if (!fastest && ((avg_i + 1) < ESP_ADC_OVERSAMPLING))
      {
        while ((uint32_t)(micros() - sub_time_target) < sub_sample_period_micro) {};
      }
    }

    data_buf[data_buf_pointer_write++] = ch1_sum / ESP_ADC_OVERSAMPLING;
    data_buf[data_buf_pointer_write++] = ch2_sum / ESP_ADC_OVERSAMPLING;

    if (!fastest) while ((uint32_t)(micros() - time_target) < sample_period_micro) {};
  } while (data_buf_pointer_write < samples);
}

static uint32_t ESP32_ADC_Read_Data(bool fastest = false)
{
  if ((data_buf == nullptr) || (data_buf_size == 0))
  {
    LastError("ADC data buffer not available");
    ESP_AvailableSampleCount = 0;
    return 0;
  }

  int ADC_GPIO = ESP_ADC1Port;
  if (ESP_ADC2) ADC_GPIO = ESP_ADC2Port;

  unsigned long samples = ESP_SampleSize;
  if (ESP_Both) samples = samples + samples;
  if (samples > data_buf_size) samples = data_buf_size;
  if (ESP_Both && ((samples % 2) != 0)) samples--;
  uint32_t sample_period_micro = 1000000 / ESP_SamplingRate;

  data_buf_pointer_write = 0;
  unsigned long time_target;

  if (!ESP32_ADC_Trigger(ESP_TriggerPort, ESP_TriggerEvent))
  {
    ESP_AvailableSampleCount = 0;
    return 0;
  }

  unsigned long total_time = micros();

  if (!ESP_Both)
  {
    ESP32_ADC_Read_Analog_To_Buffer(ADC_GPIO, samples, sample_period_micro, fastest);
  }
  else
  {
    int both_channel_key = ESP_FirstChannel * 100 + ESP_SecondChannel;
    switch (both_channel_key)
    {
      case 3535: ESP32_ADC_Read_Analog_Analog_To_Buffer(ESP_ADC1Port, ESP_ADC1Port, samples, sample_period_micro, fastest); break;
      case 3536: ESP32_ADC_Read_Analog_Analog_To_Buffer(ESP_ADC1Port, ESP_ADC2Port, samples, sample_period_micro, fastest); break;
      case 3502: ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC1Port, 2, true, samples, sample_period_micro, fastest); break;
      case 3505: ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC1Port, ESP_TriggerPort, true, samples, sample_period_micro, fastest); break;
      case 3635: ESP32_ADC_Read_Analog_Analog_To_Buffer(ESP_ADC2Port, ESP_ADC1Port, samples, sample_period_micro, fastest); break;
      case 3636: ESP32_ADC_Read_Analog_Analog_To_Buffer(ESP_ADC2Port, ESP_ADC2Port, samples, sample_period_micro, fastest); break;
      case 3602: ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC2Port, 2, true, samples, sample_period_micro, fastest); break;
      case 3605: ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC2Port, ESP_TriggerPort, true, samples, sample_period_micro, fastest); break;
      case 235:  ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC1Port, 2, false, samples, sample_period_micro, fastest); break;
      case 236:  ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC2Port, 2, false, samples, sample_period_micro, fastest); break;
      case 202:  ESP32_ADC_Read_Both_To_Buffer(ESP32_ADC_Read_Digital_2, ESP32_ADC_Read_Digital_2, samples, sample_period_micro, fastest); break;
      case 205:  ESP32_ADC_Read_Both_To_Buffer(ESP32_ADC_Read_Digital_2, ESP32_ADC_Read_Digital_Trigger, samples, sample_period_micro, fastest); break;
      case 535:  ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC1Port, ESP_TriggerPort, false, samples, sample_period_micro, fastest); break;
      case 536:  ESP32_ADC_Read_Analog_Digital_To_Buffer(ESP_ADC2Port, ESP_TriggerPort, false, samples, sample_period_micro, fastest); break;
      case 502:  ESP32_ADC_Read_Both_To_Buffer(ESP32_ADC_Read_Digital_Trigger, ESP32_ADC_Read_Digital_2, samples, sample_period_micro, fastest); break;
      case 505:  ESP32_ADC_Read_Both_To_Buffer(ESP32_ADC_Read_Digital_Trigger, ESP32_ADC_Read_Digital_Trigger, samples, sample_period_micro, fastest); break;
      default:
        LastError("Invalid ADC channel configuration");
        ESP_AvailableSampleCount = 0;
        return 0;
    }
  }

  ESP_AvailableSampleCount = data_buf_pointer_write;
  return micros() - total_time;
}

static int ESP_decode_event(String event)
{
  event.toUpperCase();
  for (int x = 0; x <= 5; x++)
  {
    if (event.equals(ESP_TriggerEvents[x])) return x;
  }
  return ESP_DefaultTriggerEvent;
}

static bool ESP32_ADC_init()
{
  ESP32_ADC_FastAttachPin(ESP_ADC1Port);
  ESP32_ADC_FastAttachPin(ESP_ADC2Port);
  ESP32_ADC_FastSetAttenuation(ADC_11db);
  ESP32_ADC_FastReadResolution(12);
  ESP32_ADC_active = true;
  return true;
}

static void ESP_Set_Port()
{
  String param_val = GetParameterValue("PORT", UpperCase);
  if ((param_val == "NOPARAM") || (param_val == "NOVAL")) return;

  if (param_val == "BOTH")
  {
    ESP_Both = true;
    ESP_ADC1 = false;
    ESP_ADC2 = false;
    return;
  }

  int temp = param_val.toInt();
  if (temp == ESP_ADC1Port)
  {
    ESP_ADC1 = true;
    ESP_ADC2 = false;
    ESP_Both = false;
    return;
  }

  if (temp == ESP_ADC2Port)
  {
    ESP_ADC1 = false;
    ESP_ADC2 = true;
    ESP_Both = false;
    return;
  }

  LastError("No valid port found, set to default (ADC Port 35)");
  ESP_ADC1 = true;
  ESP_ADC2 = false;
  ESP_Both = false;
}

static bool ESP_Set_Both_Channel(String parameter, int *channel)
{
  String param_val = GetParameterValue(parameter, UpperCase);
  if ((param_val == "NOPARAM") || (param_val == "NOVAL")) return true;

  int temp = param_val.toInt();
  if (ESP32_ADC_ChannelValid(temp))
  {
    *channel = temp;
    if (!ESP32_ADC_ChannelAnalog(temp)) pinMode(temp, INPUT);
    return true;
  }

  LastError(parameter + " must be 35, 36, 2 or 5");
  return false;
}

static void ESP_Apply_Attenuation(String param_val)
{
  bool val_valid = false;
  if (param_val == "0DB")   { ESP32_ADC_FastSetAttenuation(ADC_0db);   val_valid = true; }
  if (param_val == "2.5DB") { ESP32_ADC_FastSetAttenuation(ADC_2_5db); val_valid = true; }
  if (param_val == "6DB")   { ESP32_ADC_FastSetAttenuation(ADC_6db);   val_valid = true; }
  if (param_val == "11DB")  { ESP32_ADC_FastSetAttenuation(ADC_11db);  val_valid = true; }

  if (val_valid) ESP_Attenuation = param_val;
  else LastError("No valid attenuation value found");
}

static void ESP_Configure()
{
  String param_val;

  ESP_Set_Port();
  ESP_Set_Both_Channel("FIRSTCHANNEL", &ESP_FirstChannel);
  ESP_Set_Both_Channel("SECONDCHANNEL", &ESP_SecondChannel);

  param_val = GetParameterValue("MEASURINGINTERVAL", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL"))
  {
    ESP_MeasuringInterval = param_val.toInt();
    if ((ESP_MeasuringInterval > ESP_MeasuringIntervalMax) || (ESP_MeasuringInterval <= 0))
    {
      LastError("MeasuringInterval out of range, set to default");
      ESP_MeasuringInterval = ESP_MeasuringIntervalDefault;
    }
  }

  param_val = GetParameterValue("SAMPLINGRATE", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL"))
  {
    ESP_SamplingRate = param_val.toInt();
    if ((ESP_SamplingRate > ESP_SamplingRateMax) || (ESP_SamplingRate <= 0))
    {
      LastError("SamplingRate out of range, set to default");
      ESP_SamplingRate = ESP_SamplingRateDefault;
    }
  }


  param_val = GetParameterValue("SAMPLESIZE", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL"))
  {
    ESP_SampleSize = param_val.toInt();
    if ((ESP_SampleSize > ESP_SampleSizeMax) || (ESP_SampleSize <= 0))
    {
      LastError("SampleSize out of range, set to default");
      ESP_SampleSize = ESP_SampleSizeDefault;
    }
  }

  param_val = GetParameterValue("TRIGGEREVENT", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL")) ESP_TriggerEvent = ESP_decode_event(param_val);

  param_val = GetParameterValue("FORMAT", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL"))
  {
    if (param_val == "BYTE") ESP_Byte = true;
    else if (param_val == "STRING") ESP_Byte = false;
    else LastError("No valid format found (BYTE or STRING)");
  }

  param_val = GetParameterValue("MARKCHANNELTWO", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL")) ESP_MarkChannelTwo = ((param_val == "TRUE") || (param_val == "1"));

  param_val = GetParameterValue("ATTENUATION", UpperCase);
  if ((param_val != "NOPARAM") && (param_val != "NOVAL")) ESP_Apply_Attenuation(param_val);
}

String ESP_ADC()
{
  if (!ESP32_ADC_active) ESP32_ADC_init();
  if (!ESP32_ADC_active)
  {
    LastError("ESP32 ADC not ready");
    return "-";
  }

  if (command == "INITIALIZE")
  {
    if (ESP32_ADC_init()) return "";
    LastError("ESP32 ADC not ready");
    return "-";
  }

  if ((command == "CONFIGURE") || (command == "CONFIGUREBUFFEREDREAD"))
  {
    ESP_Configure();
    return "";
  }

  if (command == "SETATTENUATION")
  {
    String val = GetParameterValue("FACTOR", UpperCase);
    if ((val == "NOPARAM") || (val == "NOVAL")) val = ESP_AttenuationDefault;
    ESP_Apply_Attenuation(val);
    return "";
  }

  if (command == "GETCONFIGURATION")
  {
    String param_val = GetParameterValue("BUFFERSIZE", UpperCase);
    if (param_val != "NOPARAM")
    {
      print2serial(String(data_buf_size));
      return "";
    }

    print2serial("ADC Buffer Size (samples):          " + String(data_buf_size));
    print2serial("Ports in use (GPIO35/GPIO36/Both):  " + String(ESP_ADC1) + "/" + String(ESP_ADC2) + "/" + String(ESP_Both));
    print2serial("Both output channels (First/Second): " + String(ESP_FirstChannel) + "/" + String(ESP_SecondChannel));
    print2serial("SamplingRate:                       " + String(ESP_SamplingRate));
    print2serial("Oversampling:                       " + String(ESP_ADC_OVERSAMPLING));
    print2serial("SampleSize:                         " + String(ESP_SampleSize));
    print2serial("MeasuringInterval (only Envelope):  " + String(ESP_MeasuringInterval));
    print2serial("Trigger GPIO port:                  " + String(ESP_TriggerPort));
    print2serial("Trigger Event:                      " + String(ESP_TriggerEvent));
    print2serial("Attenuation:                        " + ESP_Attenuation);
    print2serial("Output in Byte ('1' = true):        " + String(ESP_Byte));
    print2serial("Mark Channel Two (bit 12 = '1'):    " + String(ESP_MarkChannelTwo));
    return "";
  }

  if (command == "GETVALUE")
  {
    bool save_ESPBoth = ESP_Both;
    bool save_ADC1 = ESP_ADC1;
    bool save_ADC2 = ESP_ADC2;
    int save_first = ESP_FirstChannel;
    int save_second = ESP_SecondChannel;

    String p = GetParameterValue("PORT", UpperCase);
    if ((p != "NOPARAM") && (p != "NOVAL"))
    {
      if (p == "BOTH")
      {
        ESP_Both = true;
        ESP_ADC1 = false;
        ESP_ADC2 = false;
        ESP_FirstChannel = ESP_ADC1Port;
        ESP_SecondChannel = ESP_ADC2Port;
      }
      else if (p.toInt() == ESP_ADC2Port)
      {
        ESP_Both = false;
        ESP_ADC1 = false;
        ESP_ADC2 = true;
      }
      else if (p.toInt() == ESP_ADC1Port)
      {
        ESP_Both = false;
        ESP_ADC1 = true;
        ESP_ADC2 = false;
      }
      else
      {
        LastError("Invalid Port for GetValue; using configured port");
      }
    }

    String result = "";
    if (ESP_Both)
    {
      uint16_t ADC1Val = ESP32_ADC_Read_Channel(ESP_FirstChannel);
      uint16_t ADC2Val = ESP32_ADC_Read_Channel(ESP_SecondChannel);
      if (ESP_Byte)
      {
        write2serial((ADC1Val >> 8) & 0xff);
        write2serial(ADC1Val & 0xff);
        write2serial((ADC2Val >> 8) & 0xff);
        write2serial(ADC2Val & 0xff);
      }
      else
      {
        result = String(ADC1Val) + "," + String(ADC2Val);
      }
    }
    else
    {
      int ADC_GPIO = ESP_ADC1Port;
      if (ESP_ADC2) ADC_GPIO = ESP_ADC2Port;

      pinMode(ADC_GPIO, INPUT);
      uint16_t ADCVal = ESP32_ADC_READ_ANALOG_AVERAGE(ADC_GPIO);
      if (ESP_Byte)
      {
        write2serial((ADCVal >> 8) & 0xff);
        write2serial(ADCVal & 0xff);
      }
      else
      {
        result = String(ADCVal);
      }
    }

    ESP_Both = save_ESPBoth;
    ESP_ADC1 = save_ADC1;
    ESP_ADC2 = save_ADC2;
    ESP_FirstChannel = save_first;
    ESP_SecondChannel = save_second;
    return result;
  }

  if (command == "GETENVELOPE")
  {
    bool save_ESPBoth = ESP_Both;
    bool save_ADC1 = ESP_ADC1;
    bool save_ADC2 = ESP_ADC2;
    int save_first = ESP_FirstChannel;
    int save_second = ESP_SecondChannel;
    unsigned long save_interval = ESP_MeasuringInterval;

    String duration = GetParameterValue("DURATION", UpperCase);
    if ((duration != "NOPARAM") && (duration != "NOVAL"))
    {
      unsigned long parsed = duration.toInt();
      if ((parsed > 0) && (parsed <= ESP_MeasuringIntervalMax)) ESP_MeasuringInterval = parsed;
      else LastError("Duration out of range, using configured MeasuringInterval");
    }

    String p = GetParameterValue("PORT", UpperCase);
    if ((p != "NOPARAM") && (p != "NOVAL"))
    {
      if (p == "BOTH")
      {
        ESP_Both = true;
        ESP_ADC1 = false;
        ESP_ADC2 = false;
        ESP_FirstChannel = ESP_ADC1Port;
        ESP_SecondChannel = ESP_ADC2Port;
      }
      else if (p.toInt() == ESP_ADC2Port)
      {
        ESP_Both = false;
        ESP_ADC1 = false;
        ESP_ADC2 = true;
      }
      else if (p.toInt() == ESP_ADC1Port)
      {
        ESP_Both = false;
        ESP_ADC1 = true;
        ESP_ADC2 = false;
      }
      else
      {
        LastError("Invalid Port for GetEnvelope; using configured port");
      }
    }

    String result = ESP32_ADC_GetEnvelope();

    ESP_Both = save_ESPBoth;
    ESP_ADC1 = save_ADC1;
    ESP_ADC2 = save_ADC2;
    ESP_FirstChannel = save_first;
    ESP_SecondChannel = save_second;
    ESP_MeasuringInterval = save_interval;
    return result;
  }

  if (command == "ACQUIRETOBUFFER")
  {
    uint32_t time_spent = ESP32_ADC_Read_Data();
    return String(time_spent);
  }

  if (command == "ACQUIRETOBUFFERMAXIMUMSPEED")
  {
    uint32_t time_spent = ESP32_ADC_Read_Data(true);
    return String(time_spent);
  }

  if (command == "GETBUFFER")
  {
    bool as_bytes = ESP_Byte;
    String f = GetParameterValue("FORMAT", UpperCase);
    if ((f != "NOPARAM") && (f != "NOVAL"))
    {
      if (f == "BYTE") as_bytes = true;
      else if (f == "STRING") as_bytes = false;
      else LastError("No valid format found (BYTE or STRING)");
    }
    ESP32_GetBuffer(as_bytes);
    ESP_AvailableSampleCount = 0;
    return "";
  }

  if (command == "GETAVAILABLESAMPLECOUNT")
  {
    return String(ESP_AvailableSampleCount);
  }

  if (command == "GETSTREAM")
  {
    print2serial("");
    FlushInput();
    ESP32_GetStream();
    return "";
  }

  LastError("No valid command found");
  return "-";
}
