/******************************************************************************
* Project: JDP26 DIY DJ Deck
* File: main.cpp
*
* Student Name: Owen Davis
* Team Members: Alexander Wang
*
* Course: ECE 304 – Junior Design Project (JDP26)
* Instructor: Prof. Baird Soules
*
* Date: May 11, 2026
*
* Description:
* This program is designed to run on an STM32 Nucleo F303RE board. It is the backbone of a DJ deck setup.
* DJ deck capabilities include: Volume control, variable frequency cutoff, toggleable echo, muting
*
* Hardware:
* - Microcontroller: STM32 Nucleo F303RE 
* - Key components: PEC11L-4120K-S0020 Rotary Encoder, Custom LPF PCB, Class D Audio Amp, Speaker
*
* Notes:
* The rotary encoder is finicky. Be very deliberate and slow when turning it to ensure that the right cutoff frequency is selected.
******************************************************************************/

#include <Arduino.h>
#include <HardwareTimer.h>
#include "stm32f3xx_hal.h"

HardwareTimer *audioTimer = nullptr;

uint32_t SAMPLE_RATE = 42000;

static constexpr uint32_t ENCODER_A = PB4; // pins
static constexpr uint32_t ENCODER_B = PB5;
static constexpr uint32_t ECHO_PIN = PB0;

static constexpr uint32_t ECHO_BUFFER_SIZE = 8192; // ~200ms of echo @ 42 kHz
volatile uint16_t echoBuffer[ECHO_BUFFER_SIZE];
volatile uint32_t echoWriteIdx = 0;

volatile int encoder_position = 0;       // 0-15
volatile unsigned long enc_last_time = 0;
int prev_position = 0;

float channel1DutyCylces[16] = {
  27.0, 18.5, 13.5, 9.5,
  6.4, 4.7, 3.5, 2.7,
  2.1, 1.85, 1.48, 1.23,
  1.08, 0.97, 0.9, 0.84
};

float channel2DutyCylces[16] = {
  75.0, 52.0, 36.0, 25.0,
  18.0, 14.0, 10.5, 8.0,
  6.2, 4.9, 4.1, 3.3,
  2.75, 2.35, 2.07, 1.84
};

int cutoffFrequencies[16] = {
    12000, 8722, 6339, 4607,
    3348, 2433, 1769, 1285,
    934, 679, 494, 359,
    261, 190, 138, 100
};

struct PwmChannel {
    uint32_t pin;
    HardwareTimer *timer;
    uint32_t channel;
    float dutyCyclePct;
};

static PwmChannel pwm[2] = {
  { PA8, nullptr, 0, 50.0f },
  { PA9, nullptr, 0, 50.0f },
};

bool initChannel(PwmChannel &ch){
  TIM_TypeDef *instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(ch.pin), PinMap_PWM);
    if (instance == nullptr) return false;

    ch.channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(ch.pin), PinMap_PWM));
    ch.timer = new HardwareTimer(instance);
    ch.timer->setMode(ch.channel, TIMER_OUTPUT_COMPARE_PWM1, ch.pin);
    ch.timer->setOverflow(17578, HERTZ_FORMAT);

    return true;
}

void applyDutyCycle(PwmChannel &ch, float pct){
    pct = constrain(pct, 0.0f, 100.0f);
    ch.dutyCyclePct = pct;

    uint32_t overflow = ch.timer->getOverflow(TICK_FORMAT);
    uint32_t compare  = (uint32_t)((pct / 100.0f) * (float)(overflow + 1));

    if (pct <= 0.0f) compare = 0;
    if (pct >= 100.0f) compare = overflow + 1;
    ch.timer->setCaptureCompare(ch.channel, compare, TICK_COMPARE_FORMAT);
}

int clamp(int x){
  return constrain(x, -2048, 2047);
}

void setDAC(uint32_t val){
    if (val > 4095) val = 4095;
    DAC1->DHR12R1 = (uint16_t)val;
}

void setupDAC(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DAC1_CLK_ENABLE();
    DAC1->CR = 0;
    DAC1->CR |= DAC_CR_EN1;
    GPIOA->MODER |= GPIO_MODER_MODER4;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    setDAC(2048);
}

void setupADC() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    GPIOA->MODER |= GPIO_MODER_MODER0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;

    ADC12_COMMON->CCR &= ~ADC12_CCR_CKMODE;
    ADC12_COMMON->CCR |= ADC12_CCR_CKMODE_0;

    if (ADC1->CR & ADC_CR_ADEN){
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN) {}
    }

    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |=  ADC_CR_ADVREGEN_0;
    delay(50);

    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) {}

    ADC1->CFGR = 0;
    ADC1->SMPR1 &= ~ADC_SMPR1_SMP1;
    ADC1->SMPR1 |=  ADC_SMPR1_SMP1;
    ADC1->SQR1 = 0;
    ADC1->SQR1 |= (1U << ADC_SQR1_SQ1_Pos);
    ADC1->CR |=  ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

uint16_t readADC(){
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    return (uint16_t)(ADC1->DR & 0x0FFF);
}

void audioISR(){
    uint16_t sample = readADC();
    echoBuffer[echoWriteIdx] = sample;
    echoWriteIdx = (echoWriteIdx + 1) & (ECHO_BUFFER_SIZE - 1);
    uint32_t out;
    bool echoEnabled = !(GPIOB->IDR & GPIO_IDR_0);

    if (echoEnabled) {
        uint16_t delayed = echoBuffer[echoWriteIdx];
        int live_s = (int)sample - 2048;
        int delayed_s = (int)delayed - 2048;
        int mixed_s = live_s + (int)(0.7 * (float)delayed_s);
        mixed_s = clamp(mixed_s);
        out = (uint32_t)(mixed_s + 2048);
    } else {
        out = sample;
    }
    if (GPIOB->IDR & GPIO_IDR_3) {
        setDAC(out);
    } else {
        setDAC(2048); // outputting DAC midpoint is the same as muting  
    }
}

void encoderISR(){
    if ((millis() - enc_last_time) < 300)  // debounce
        return;

    if (digitalRead(ENCODER_B) == HIGH) {
        encoder_position = constrain(encoder_position - 1, 0, 15);
    } else {
        encoder_position = constrain(encoder_position + 1, 0, 15);
    }
    enc_last_time = millis();
}

void setup(){
    Serial.begin(115200);
    delay(1000);
    Serial.print("Hello Bro. I am a DJ Deck Bro.\n");

    setupADC();
    setupDAC();

    pinMode(ECHO_PIN,  INPUT_PULLUP);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    audioTimer = new HardwareTimer(TIM3);
    audioTimer->setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
    audioTimer->attachInterrupt(audioISR);
    audioTimer->resume();

    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000);

    for (uint8_t i = 0; i < 2; i++) {
        initChannel(pwm[i]);
        applyDutyCycle(pwm[i], pwm[i].dutyCyclePct);
        pwm[i].timer->resume();
    }
}

void loop(){
    if (prev_position != encoder_position) {
        Serial.print(cutoffFrequencies[encoder_position]);
        Serial.println(" Hz");

        applyDutyCycle(pwm[0], channel1DutyCylces[encoder_position]); // set duty cycles
        applyDutyCycle(pwm[1], channel2DutyCylces[encoder_position]);

        prev_position = encoder_position;
    }
    if (digitalRead(ECHO_PIN) == 0) {
        Serial.print(" echo ");
    }
}