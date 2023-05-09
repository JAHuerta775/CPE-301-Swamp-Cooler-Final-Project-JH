//****************************************
 * Name: Jose Huerta
 * Assignment: Swamp Cooler Final Project 
 * Date: 05/09/23
 ****************************************//
 
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <TimeLib.h>
#include "DHT.h"
#include <ServoTimer2.h>
#include <DHT_U.h>
#include <RTClib.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup();
void loop();
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
void DelayTimer(long int DelayValue);
void convertToF();
time_t requestSync();
void processSyncMessage();
void printDigits(int digits);
void logTime(bool b);
void displayClimate();
void displayError();

#define PINONE A1          // Stepper Motor 1st Pin
#define PINTWO A2          // Stepper Motor 2nd Pin
#define PINTHREE A3        // Stepper Motor 3rd Pin
#define PINFOUR A4         // Stepper Motor 4th Pin
#define STEPS 32           // Motor Calc Steps 
#define DHTTYPE DHT11      // DHT for Temperature and Humidity
#define DHTPIN 7           // Pin 7 for DHT
#define TIME_HEADER  'T'   
#define TIME_REQUEST  7   

Stepper stepper(STEPS,PINONE,PINTWO,PINTHREE,PINFOUR);
RTC_DS1307 rtc;
DHT mySensor(DHTPIN,DHTTYPE);
LiquidCrystal lcd(6,5,4,3,8,2);


volatile unsigned char * my_ADMUX = (unsigned char *) 0x7C;  
volatile unsigned char * my_ADCSRB = (unsigned char *) 0x7B;
volatile unsigned char * my_ADCSRA = (unsigned char *) 0x7A;
volatile unsigned int * my_ADC_DATA = (unsigned int *) 0x78;
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;
volatile unsigned char* ddr_k  = (unsigned char*) 0x107; 
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* pin_l = (unsigned char*) 0x109;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;

// Position of Vents
int vent_open = 0;
int vent_close = 90;

int watersensor_id = 0;
volatile unsigned int historyValue;
const int waterThreshold = 125;
const int tempThreshold = 60;

float Temp = 0;
float Humidity = 0;

//Vent Control
ServoTimer2 vent_control;

bool temperatureabovelevel = false;
bool Fan = false;
bool Idle = false;
bool Error = false;
bool Regain = false;
bool watergood = false;

ISR(TIMER3_COMPA_vect)
{
  int currentValue = adc_read(watersensor_id); 
  historyValue = currentValue;

  if((currentValue < waterThreshold) && (watergood == true)) { Error = true; }
  if(!(currentValue < waterThreshold) && (watergood == false)) { Regain = true; }
  if(currentValue < waterThreshold) { watergood = false;}
  else { watergood = true; }
  
  if(Temp > tempThreshold) { temperatureabovelevel = true; }
  else { temperatureabovelevel = false; }
}

void setup() {
  
  *ddr_b |= 0b11110000;
  *ddr_k &= 0b01111111;
  *port_k |= 0b10000000;
  *ddr_l |= 0b00101000;
  // LOW
  *port_l &= 0b11010111;
  // Start vents
  vent_control.attach(9);
  vent_control.write(90);

  Serial.begin(9600);
  setSyncProvider(requestSync); 
  lcd.begin(16, 2);
  lcd.print("Reading");
  lcd.setCursor(0,2);
  lcd.print("Climate");

  // OCR0A = 255;
  // TCCR0A = (1 << WGM01); 
  // TCCR0B = 0x01;
  // TIMSK0 = (1 << OCIE0A); /
  // sei();                  
  
  OCR3A = 255;
  TCCR3A = (1 << WGM01); 
  TCCR3B = 0x03;
  TIMSK3 = (1 << OCIE0A); 
  sei();               
}

void loop() {

  if(Serial.available()) { processSyncMessage(); }
  if(!(*pin_k & 0b10000000))
  {
    for (volatile unsigned int i = 0; i < 50; i++); 
      if(!(*pin_k & 0b10000000))
      {
        Idle = !Idle;
        if(Idle)
          {
            cli();
          }
        else
          {
            *port_b &= 0b11101111;
            sei();
          }
        while(!(*pin_k & 0b10000000));
      }
  }

  if(Idle)
  {
    *port_b |= 0b00010000; // YELLOW LED ON
    *port_b &= 0b00011111; // OTHER LEDS OFF
    //*port_h &= 0b10111111;
    if(Fan) { logTime(!Fan);
                *port_l &= 0b11110111; }  
    Fan = false;
  //  FAN OFF
    vent_control.write(0);
  }
  else
  {
    if(!watergood)
    {
    *port_b |= 0b00100000; // RED LED ON
    *port_b &= 0b00111111; // GREEN/BLUE LED OFF
    //*port_h &= 0b10111111; //BLUE LED OFF
    if(Fan) { logTime(!Fan);
                *port_l &= 0b11110111; } // send a log saying fan was disabled if it was on
                                         // FAN OFF
    Fan = false;
    displayError();
    // Vent closes
    vent_control.write(0);   
    }
    else if(watergood)
    {
      *port_b &= 0b11011111; // RED LED OFF
      if(Regain)
      {
        if(mySensor.readTemperature() > 0)
        {
          Temp = (mySensor.readTemperature() * 9/5) + 32;
          Humidity = mySensor.readHumidity();
        }
        
          lcd.clear();
          lcd.print("Temp: ");
          lcd.print(Temp);
          lcd.print(" F");
          lcd.setCursor(0,2);
          lcd.print("Humidity: ");
          lcd.print(Humidity);
          lcd.print("%");
          Regain = false;
      }

      displayClimate();
      if(!temperatureabovelevel){ *port_b |= 0b01000000; } // GREEN LED ON

      if(temperatureabovelevel)
      {
        *port_b &= 0b10111111; // GREEN LED OFF
        *port_b |= 0b10000000; // BLUE LED ON
        //*port_h |= 0b01000000; // BLUE LED ON
        if(!Fan) { logTime(true); 
                      *port_l |= 0b00001000; } 
        Fan = true;
        //Vent opens
        vent_control.write(180);
      }
      else
      {
        *port_h &= 0b10111111; // BLUE LED ON
        if(Fan) { logTime(!Fan); 
                    *port_l &= 0b11110111; } 
        Fan =  false;
        // VENT DISABLED
        vent_control.write(0);
      }
    }
  }
}

void displayClimate()
{
  mySensor.read();
  if(Regain)
  {
      if(mySensor.readTemperature() > 0)
      {
        Temp = (mySensor.readTemperature() * 9/5) + 32;
        Humidity = mySensor.readHumidity();
      }

    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(Temp);
    lcd.print(" F");
    lcd.setCursor(0,2);
    lcd.print("Humidity: ");
    lcd.print(Humidity);
    lcd.print("%");
    Regain = false;
  } else if((mySensor.readTemperature() > 0) && watergood)
  {
    Temp = (mySensor.readTemperature() * 9/5) + 32;
    Humidity = mySensor.readHumidity();
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(Temp);
    lcd.print(" F");
    lcd.setCursor(0,2);
    lcd.print("Humidity: ");
    lcd.print(Humidity);
    lcd.print("%");
  } else if((Temp == 0) && watergood)
  {
    lcd.clear();
    lcd.print("Reading");
    lcd.setCursor(0,2);
    lcd.print("Climate");
  }

}

void displayError()
{
  if(Error)
  {
    lcd.clear();
    lcd.print("Error!");
    lcd.setCursor(0,2);
    lcd.print("Reservior Empty!");
    Error = false;
  }
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  uint8_t low, high;

  if (adc_channel_num >= 54) adc_channel_num -= 54;

  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adc_channel_num >> 3) & 0x01) << MUX5);

  ADMUX = (1 << 6) | (adc_channel_num & 0x07);

#if defined(ADCSRA) && defined(ADCL)
  sbi(ADCSRA, ADSC);

  // ADSC is cleared
  while (bit_is_set(ADCSRA, ADSC));
  
  low  = ADCL;
  high = ADCH;
#else
  // returns a 0 if no data
  low  = 0;
  high = 0;
#endif

  return (high << 8) | low;
}

void printDigits(int digits)
{
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

time_t requestSync()
{
  Serial.write(TIME_REQUEST);  
  return 0; 
}

void processSyncMessage() 
{
  unsigned long pctime;
  const unsigned long DEFAULT_TIME = 1357041600; 

  if(Serial.find(TIME_HEADER)) {
     pctime = Serial.parseInt();
     if( pctime >= DEFAULT_TIME) { //
       setTime(pctime); 
     }
  }
}

void logTime(bool b)
{
  if(b) { Serial.print("Fan On\t- "); }
  else { Serial.print("Fan Off\t- "); }

  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.print(year()); 
  Serial.println(); 
}
