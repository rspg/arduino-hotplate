#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

#define OLED_ADDRESS    (0x3C)
#define PIN_ROTALY_ENCODER_A 2
#define PIN_ROTALY_ENCODER_B 3
#define PIN_THERMISTOR_SENSE_1 0
#define PIN_THERMISTOR_SENSE_2 1

unsigned char RotalyEncoderStateR = 0;
unsigned char RotalyEncoderStateL = 0;
char RotalyEncoderDelta = 0;
SSD1306AsciiAvrI2c oled;

void setup() 
{
    pinMode(PIN_ROTALY_ENCODER_A, INPUT_PULLUP);
    pinMode(PIN_ROTALY_ENCODER_B, INPUT_PULLUP);
    analogReference(INTERNAL);

    oled.begin(&Adafruit128x32, OLED_ADDRESS);
    oled.setFont(CalLite24);
    //oled.displayRemap(true);

    attachInterrupt(0, onRotalyEncoder, CHANGE);
    attachInterrupt(1, onRotalyEncoder, CHANGE);

    Serial.begin(9600);
}

void onRotalyEncoder()
{
    bool valueA = digitalRead(PIN_ROTALY_ENCODER_A);
    bool valueB = digitalRead(PIN_ROTALY_ENCODER_B);

    switch(RotalyEncoderStateR)
    {
        case 0:
            if(!valueA && valueB)
                RotalyEncoderStateR = 1;
            break;
        case 1:
            if(valueA && valueB)
                RotalyEncoderStateR = 2;
            break;
        case 2:
            if(valueA && !valueB)
                RotalyEncoderStateR = 3;
            break;
        case 3:
            if(!valueA && !valueB)
            {
                RotalyEncoderStateR = 0;
                RotalyEncoderStateL = 0;
                RotalyEncoderDelta++;
            }
            break;
    }

    switch(RotalyEncoderStateL)
    {
        case 0:
            if(valueA && valueB)
                RotalyEncoderStateL = 1;
            break;
        case 1:
            if(!valueA && valueB)
                RotalyEncoderStateL = 2;
            break;
        case 2:
            if(!valueA && !valueB)
                RotalyEncoderStateL = 3;
            break;
        case 3:
            if(valueA && !valueB)
            {
                RotalyEncoderStateR = 0;
                RotalyEncoderStateL = 0;
                RotalyEncoderDelta--;
            }
            break;
    }
}

float getTemperature(uint8_t pin)
{
    const float B = 3950.f;
    const float V = 5.0f;
    const float Vref = 1.1f;
    const float Rp = 30.f;
    const float T0 = 25 + 273.15f;
    const float R0 = 100*1000.f;

    float voltage = analogRead(pin) * Vref / 1024.0;
    if(voltage <= 0.f)
        return 0;
    float resistance = -Rp*(voltage - V)/voltage;
    float temperature = B*T0/(logf(resistance/R0)*T0+B) - 273.15f;
    return temperature;
}

void loop()
{
    Serial.print("Temperature 1: ");
    Serial.println(getTemperature(PIN_THERMISTOR_SENSE_1));
    Serial.print("Temperature 2: ");
    Serial.println(getTemperature(PIN_THERMISTOR_SENSE_2));
    
    oled.home();
    oled.print((int)RotalyEncoderDelta);

    delay(1);
}
