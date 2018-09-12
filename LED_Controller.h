#define CH1_REDPIN 5
#define CH1_GREENPIN 6
#define CH1_BLUEPIN 3
#define CH2_REDPIN 10
#define CH2_GREENPIN 11
#define CH2_BLUEPIN 9

typedef struct
{
  byte r;
  byte g;
  byte b;
} RgbColor;

typedef struct
{
  uint16_t hue;
  byte saturation;
  byte value;
} HsvColor;

typedef struct
{
  double h;
  double s;
  double v;
} Hsv;

RgbColor HsvToRgb(HsvColor hsv);





#define SOFTPWM_OUTPUT_DELAY

// PalatisSoftPWM - software PWM library for Arduino: https://github.com/per1234/PalatisSoftPWM
#ifndef PalatisSoftPWM_h
#define PalatisSoftPWM_h

#include <Arduino.h>

#ifndef __AVR__
#error Architecture not supported. PalatisSoftPWM currently only supports AVR microcontrollers.
#endif

// helper macros
#define SOFTPWM_DEFINE_PINMODE(CHANNEL, PMODE, PORT, BIT) \
  template <> \
  inline void pinModeStatic<CHANNEL>(const uint8_t mode) { \
    if (mode == INPUT) { \
      bitClear(PMODE, BIT); \
      bitClear(PORT, BIT); \
    } \
    else if (mode == INPUT_PULLUP) { \
      bitClear(PMODE, BIT); \
      bitSet(PORT, BIT); \
    } \
    else { \
      bitSet(PMODE, BIT); \
    } \
  }




// arduino:standard or arduino:eightanalogoutputs variants
#define SOFTPWM_DEFINE_PIN0_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD0)
#define SOFTPWM_DEFINE_PIN0_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD0)
#define SOFTPWM_DEFINE_PIN1_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD1)
#define SOFTPWM_DEFINE_PIN1_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD1)
#define SOFTPWM_DEFINE_PIN2_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD2)
#define SOFTPWM_DEFINE_PIN2_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD2)
#define SOFTPWM_DEFINE_PIN3_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD3)
#define SOFTPWM_DEFINE_PIN3_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD3)
#define SOFTPWM_DEFINE_PIN4_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD4)
#define SOFTPWM_DEFINE_PIN4_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD4)
#define SOFTPWM_DEFINE_PIN5_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD5)
#define SOFTPWM_DEFINE_PIN5_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD5)
#define SOFTPWM_DEFINE_PIN6_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD6)
#define SOFTPWM_DEFINE_PIN6_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD6)
#define SOFTPWM_DEFINE_PIN7_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRD, PORTD, PORTD7)
#define SOFTPWM_DEFINE_PIN7_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRD, PORTD, PORTD7)
#define SOFTPWM_DEFINE_PIN8_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB0)
#define SOFTPWM_DEFINE_PIN8_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB0)
#define SOFTPWM_DEFINE_PIN9_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB1)
#define SOFTPWM_DEFINE_PIN9_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB1)
#define SOFTPWM_DEFINE_PIN10_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB2)
#define SOFTPWM_DEFINE_PIN10_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB2)
#define SOFTPWM_DEFINE_PIN11_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB3)
#define SOFTPWM_DEFINE_PIN11_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB3)
#define SOFTPWM_DEFINE_PIN12_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB4)
#define SOFTPWM_DEFINE_PIN12_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB4)
#define SOFTPWM_DEFINE_PIN13_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRB, PORTB, PORTB5)
#define SOFTPWM_DEFINE_PIN13_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRB, PORTB, PORTB5)
#define SOFTPWM_DEFINE_PIN14_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC0)
#define SOFTPWM_DEFINE_PIN14_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC0)
#define SOFTPWM_DEFINE_PIN15_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC1)
#define SOFTPWM_DEFINE_PIN15_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC1)
#define SOFTPWM_DEFINE_PIN16_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC2)
#define SOFTPWM_DEFINE_PIN16_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC2)
#define SOFTPWM_DEFINE_PIN17_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC3)
#define SOFTPWM_DEFINE_PIN17_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC3)
#define SOFTPWM_DEFINE_PIN18_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC4)
#define SOFTPWM_DEFINE_PIN18_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC4)
#define SOFTPWM_DEFINE_PIN19_CHANNEL(CHANNEL) SOFTPWM_DEFINE_CHANNEL(CHANNEL, DDRC, PORTC, PORTC5)
#define SOFTPWM_DEFINE_PIN19_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_CHANNEL_INVERT(CHANNEL, DDRC, PORTC, PORTC5)
#define SOFTPWM_DEFINE_PINA0_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN14_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA0_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN14_CHANNEL_INVERT(CHANNEL)
#define SOFTPWM_DEFINE_PINA1_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN15_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA1_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN15_CHANNEL_INVERT(CHANNEL)
#define SOFTPWM_DEFINE_PINA2_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN16_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA2_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN16_CHANNEL_INVERT(CHANNEL)
#define SOFTPWM_DEFINE_PINA3_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN17_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA3_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN17_CHANNEL_INVERT(CHANNEL)
#define SOFTPWM_DEFINE_PINA4_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN18_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA4_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN18_CHANNEL_INVERT(CHANNEL)
#define SOFTPWM_DEFINE_PINA5_CHANNEL(CHANNEL) SOFTPWM_DEFINE_PIN19_CHANNEL(CHANNEL)
#define SOFTPWM_DEFINE_PINA5_CHANNEL_INVERT(CHANNEL) SOFTPWM_DEFINE_PIN19_CHANNEL_INVERT(CHANNEL)





#define SOFTPWM_DEFINE_CHANNEL(CHANNEL, PMODE, PORT, BIT) \
  template <> \
  inline void bitWriteStatic<CHANNEL>(const bool value) { \
    bitWrite( PORT, BIT, value ); \
  } \
  SOFTPWM_DEFINE_PINMODE( CHANNEL, PMODE, PORT, BIT )

#define SOFTPWM_DEFINE_CHANNEL_INVERT( CHANNEL, PMODE, PORT, BIT ) \
  template <> \
  inline void bitWriteStatic<CHANNEL>(const bool value) { \
    bitWrite(PORT, BIT, !value); \
  } \
  SOFTPWM_DEFINE_PINMODE(CHANNEL, PMODE, PORT, BIT)

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(CHANNEL_CNT, PWM_LEVELS) \
  CSoftPWM<CHANNEL_CNT, PWM_LEVELS> PalatisSoftPWM; \
  ISR(TIM1_COMPA_vect) { \
    interrupts(); \
    PalatisSoftPWM.update(); \
  }
#else  //defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(CHANNEL_CNT, PWM_LEVELS) \
  CSoftPWM<CHANNEL_CNT, PWM_LEVELS> PalatisSoftPWM; \
  ISR(TIMER1_COMPA_vect) { \
    interrupts(); \
    PalatisSoftPWM.update(); \
  }
#endif  //defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)

#define SOFTPWM_DEFINE_OBJECT(CHANNEL_CNT) \
  SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(CHANNEL_CNT, 0)

#define SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(CHANNEL_CNT, PWM_LEVELS) \
  extern CSoftPWM<CHANNEL_CNT, PWM_LEVELS> PalatisSoftPWM;

#define SOFTPWM_DEFINE_EXTERN_OBJECT(CHANNEL_CNT) \
  SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(CHANNEL_CNT, 0)

// here comes the magic @o@
template <int channel> inline void bitWriteStatic(bool value) {}
template <int channel> inline void pinModeStatic(uint8_t mode) {}

template <int channel>
struct bitWriteStaticExpander {
  void operator() (bool value) const {
    bitWriteStatic<channel>(value);
    bitWriteStaticExpander < channel - 1 > ()(value);
  }

  void operator() (const uint8_t &count, const uint8_t * const &channels) const {
#ifdef SOFTPWM_OUTPUT_DELAY
    bitWriteStatic<channel>((count + channel) < channels[channel]);
#else  //SOFTPWM_OUTPUT_DELAY
    bitWriteStatic<channel>(count < channels[channel]);
#endif  //SOFTPWM_OUTPUT_DELAY
    bitWriteStaticExpander < channel - 1 > ()(count, channels);
  }
};

template <>
struct bitWriteStaticExpander < -1 > {
  void operator() (bool) const {}
  void operator() (const uint8_t &, const uint8_t* const &) const {}
};

template <int channel>
struct pinModeStaticExpander {
  void operator() (const uint8_t mode) const
  {
    pinModeStatic<channel>(mode);
    pinModeStaticExpander < channel - 1 > ()(mode);
  }
};

template <>
struct pinModeStaticExpander < -1 > {
  void operator() (const uint8_t) const {}
};

template <unsigned int num_channels, unsigned int num_PWM_levels>
class CSoftPWM {
  public:
    void begin(const unsigned long hertz) {
      allOff();  //this prevents inverted channels from momentarily going LOW
      asm volatile ("/************ pinModeStaticExpander begin ************/");
      const uint8_t oldSREG = SREG;
      noInterrupts();
      pinModeStaticExpander < num_channels - 1 > ()( OUTPUT );
      SREG = oldSREG;
      asm volatile ("/************ pinModeStaticExpander end ************/");

      /* the setup of timer1 is stolen from ShiftPWM :-P
         http://www.elcojacobs.com/shiftpwm/ */
      asm volatile ("/************ timer setup begin ************/");
      TCCR1A = 0b00000000;
      TCCR1B = 0b00001001;
      OCR1A = (F_CPU - hertz * PWMlevels() / 2) / (hertz * PWMlevels());
      bitSet(TIMSK1, OCIE1A);
      asm volatile ("/************ timer setup end ************/");

      _count = 0;
    }

    void set(const int channel_idx, const uint8_t value) {
      _channels[channel_idx] = value;
    }

    size_t size() const {
      return num_channels;
    }

    unsigned int PWMlevels() const {
      return num_PWM_levels ? num_PWM_levels : 256;
    }

    void allOff() {
      asm volatile ("/********** CSoftPWM::allOff() begin **********/");
      const uint8_t oldSREG = SREG;
      noInterrupts();
      for (int i = 0; i < num_channels; i++) {
        _channels[i] = 0;
      }
      bitWriteStaticExpander < num_channels - 1 > ()(false);
      SREG = oldSREG;
      asm volatile ("/********** CSoftPWM::allOff() end **********/");
    }

    /* This function cannot be private because the ISR uses it, and I have
       no idea about how to make friends with ISR. :-( */
    void update() __attribute__((always_inline)) {
      asm volatile ("/********** CSoftPWM::update() begin **********/");
      const uint8_t count = _count;
      bitWriteStaticExpander < num_channels - 1 > ()(count, _channels);
      _count++;
      if (_count == PWMlevels()) {
        _count = 0;
      }
      asm volatile ("/********** CSoftPWM::update() end **********/");
    }

    /* this function is stolen from ShiftPWM :-P
       http://www.elcojacobs.com/shiftpwm/ */
    void printInterruptLoad() {
      unsigned long time1, time2;

      bitSet(TIMSK1, OCIE1A); // enable interrupt
      time1 = micros();
      delayMicroseconds(5000);
      time1 = micros() - time1;

      bitClear(TIMSK1, OCIE1A); // disable interrupt
      time2 = micros();
      delayMicroseconds(5000);
      time2 = micros() - time2;

      const double load = static_cast<double>(time1 - time2) / time1;
      const double interrupt_frequency = static_cast<double>(F_CPU) / (OCR1A + 1);

      Serial.println(F("PalatisSoftPWM::printInterruptLoad():"));
      Serial.print(F("  Load of interrupt: "));
      Serial.println(load, 10);
      Serial.print(F("  Clock cycles per interrupt: "));
      Serial.println(load * F_CPU / interrupt_frequency);
      Serial.print(F("  Interrupt frequency: "));
      Serial.print(interrupt_frequency);
      Serial.println(F(" Hz"));
      Serial.print(F("  PWM frequency: "));
      Serial.print(interrupt_frequency / PWMlevels());
      Serial.println(F(" Hz"));
      Serial.print(F("  PWM levels: "));
      Serial.println(PWMlevels());

      bitSet(TIMSK1, OCIE1A);  // enable interrupt again
    }

  private:
    uint8_t _channels[num_channels];
    uint8_t _count;
};

#endif //PalatisSoftPWM_h
