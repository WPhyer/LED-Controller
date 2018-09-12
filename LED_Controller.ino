#include "LED_Controller.h"

RgbColor rgbColor1;
RgbColor rgbColor2;
HsvColor hsvColor1;
HsvColor hsvColor2;

bool isLED;
bool hasStart;
bool hasStop;
bool isReset;
byte channel; // only support for 2 channels - 1 or 2 - 0 means no active channel
byte byteRead;
int curPos;
int rxCount;
int twoBytes;
byte rxBuffer[12];
byte f1[2];
byte f4[2];
String tmpBuffer;



#define SOFTPWM

#ifdef SOFTPWM
SOFTPWM_DEFINE_PIN3_CHANNEL(0); //, DDRD, PORTD, PORTD3);
SOFTPWM_DEFINE_PIN5_CHANNEL(1); //, DDRD, PORTD, PORTD5);
SOFTPWM_DEFINE_PIN6_CHANNEL(2); //, DDRD, PORTD, PORTD6);
SOFTPWM_DEFINE_PIN9_CHANNEL(3); //, DDRD, PORTD, PORTD9);
SOFTPWM_DEFINE_PIN10_CHANNEL(4); //, DDRD, PORTD, PORTD10);
SOFTPWM_DEFINE_PIN11_CHANNEL(5); //, DDRD, PORTD, PORTD11);

SOFTPWM_DEFINE_OBJECT(6);
//SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(6, 256);
//SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(6, 256);
#endif




void setup()
{
  /* set TIMER1 to FAST mode */
  //bitSet(TCCR1B, WGM12);
#ifdef SOFTPWM  
  PalatisSoftPWM.begin(60); // begin with 60hz pwm frequency
#endif

#ifndef SOFTPWM
  /* setup channel 1 LED pins */
  pinMode(CH1_REDPIN, OUTPUT);
  pinMode(CH1_GREENPIN, OUTPUT);
  pinMode(CH1_BLUEPIN, OUTPUT);
  /* setup channel 2 LED pins */
  pinMode(CH2_REDPIN, OUTPUT);
  pinMode(CH2_GREENPIN, OUTPUT);
  pinMode(CH2_BLUEPIN, OUTPUT);
#endif

  // Turn the Serial Protocol ON
  Serial.begin(9600);

  isLED = false;
  hasStart = false;
  hasStop = false;
  isReset = false;
  channel = '~';
  curPos = 0;
  rxCount = 0;
  twoBytes = 0;
  rgbColor1.r = 0; rgbColor1.g = 0; rgbColor1.b = 0;
  rgbColor2.r = 0; rgbColor2.g = 0; rgbColor2.b = 0;
  hsvColor1.hue = 0; hsvColor1.saturation = 0; hsvColor1.value = 0;
  hsvColor2.hue = 0; hsvColor2.saturation = 0; hsvColor2.value = 0;
  tmpBuffer = "";
}

void loop()
{
  /*  check if data has been sent */
  /*  need to read 12 bytes        */
  /*  must start with 0xF1 0xF1   */
  /*  must end with 0xF4 0xF4     */
  if (Serial.available())
  {
    isLED = false;
    
    rxCount = Serial.readBytes(rxBuffer, 12);
    
    if (rxCount == 12)
    {
      hasStart = rxBuffer[0] == 0xF1 && rxBuffer[1] == 0xF1;
      hasStop = rxBuffer[10] == 0xF4 && rxBuffer[11] == 0xF4;
      if (hasStart && hasStop)
      {
        isLED = true;
      }
    }
  }
  
  if (isLED)
  {
    memcpy(&hsvColor1, rxBuffer + 2, sizeof(HsvColor));
    memcpy(&hsvColor2, rxBuffer + 6, sizeof(HsvColor));

    rgbColor1 = HsvToRgb(hsvColor1);
    rgbColor2 = HsvToRgb(hsvColor2);

#ifndef SOFTPWM
    analogWrite(CH1_REDPIN, rgbColor1.r);
    analogWrite(CH1_BLUEPIN, rgbColor1.b);
    analogWrite(CH1_GREENPIN, rgbColor1.g);
    analogWrite(CH2_REDPIN, rgbColor2.r);
    analogWrite(CH2_BLUEPIN, rgbColor2.b);
    analogWrite(CH2_GREENPIN, rgbColor2.g);
#endif    
    
#ifdef SOFTPWM
    PalatisSoftPWM.set(0, rgbColor1.b);
    PalatisSoftPWM.set(1, rgbColor1.r);
    PalatisSoftPWM.set(2, rgbColor1.g);
    PalatisSoftPWM.set(3, rgbColor2.b);
    PalatisSoftPWM.set(4, rgbColor2.r);
    PalatisSoftPWM.set(5, rgbColor2.g);
#endif

    isLED = false;
  }
}

RgbColor HsvToRgb(HsvColor hsvColor)
{
  double r = 0, g = 0, b = 0;
  Hsv hsv;
  hsv.h = hsvColor.hue;
  hsv.s = (double)hsvColor.saturation * 0.01;
  hsv.v = (double)hsvColor.value * 0.01;

  if (hsv.s == 0)
  {
    r = hsv.v;
    g = hsv.v;
    b = hsv.v;
  }
  else
  {
    int i;
    double f, p, q, t;

    if (hsv.h == 360)
      hsv.h = 0;
    else
      hsv.h = hsv.h / 60;

    i = (int)trunc(hsv.h);
    f = hsv.h - i;

    p = hsv.v * (1.0 - hsv.s);
    q = hsv.v * (1.0 - (hsv.s * f));
    t = hsv.v * (1.0 - (hsv.s * (1.0 - f)));
  
    switch (i)
    {
      case 0:
        r = hsv.v;
        g = t;
        b = p;
        break;

      case 1:
        r = q;
        g = hsv.v;
        b = p;
        break;

      case 2:
        r = p;
        g = hsv.v;
        b = t;
        break;

      case 3:
        r = p;
        g = q;
        b = hsv.v;
        break;

      case 4:
        r = t;
        g = p;
        b = hsv.v;
        break;

      default:
        r = hsv.v;
        g = p;
        b = q;
        break;
    }

  }

  RgbColor rgb;
  rgb.r = r * 255;
  rgb.g = g * 255;
  rgb.b = b * 255;

  return rgb;
}
