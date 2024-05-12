#include <LiquidCrystal.h>
const int RS = 31, EN = 33, D4 = 35, D5 = 37, D6 = 39, D7 = 41;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

#define SPEED_PIN 3
#define DIR1 5
#define DIR2 3

#include <DHT.h>
#include <RTClib.h>

#define LCD_INTERVAL 60000

#define RDA 0x80
#define TBE 0x20

const byte interruptPin = 2;

RTC_DS1307 rtc;

#define TEMP_THRESHOLD 20
#define WATER_THRESHOLD 100
#define DHT_PIN 7

#define powerButton 18
#define resetButton 19
#define fanPIN 9


int IdleLed = 52;
volatile unsigned char *port_k = (unsigned char *)0x108; // PORTK register address
volatile unsigned char *ddr_k = (unsigned char *)0x107; // DDRK register address

int DisabledLed = 50;
volatile unsigned char *port_h = (unsigned char *)0x105; // PORTH register address
volatile unsigned char *ddr_h = (unsigned char *)0x104; // DDRH register address

int RunningLed = 48;
volatile unsigned char *port_e = (unsigned char *)0x103; // PORTE register address
volatile unsigned char *ddr_e = (unsigned char *)0x102; // DDRE register address

int ErrorLed = 46;
int Off;


void writePort(volatile unsigned char*reg, unsigned char bit, bool state){
  bit = 0x01 << bit;
  if(state){*reg |= bit;}
  else{*reg &= ~bit;}
}

volatile unsigned char *port_g = (unsigned char *)0x34;
volatile unsigned char *ddr_g = (unsigned char *)0x33;
volatile unsigned char *pin_g = (unsigned char *)0x32;


volatile unsigned char *port_b = (unsigned char *)0x25;
volatile unsigned char *ddr_b = (unsigned char *)0x24;
volatile unsigned char *pin_b = (unsigned char *)0x23;


//water level sensor
volatile unsigned char *port_k = (unsigned char *)0x108;
volatile unsigned char *ddr_k = (unsigned char *)0x107;
volatile unsigned char *pin_k = (unsigned char *)0x106;

volatile unsigned char *port_e = (unsigned char *)0x2E;
volatile unsigned char *ddr_e = (unsigned char *)0x2D;
volatile unsigned char *pin_e = (unsigned char *)0x2C;

//temperature sensor
volatile unsigned char *port_d = (unsigned char *)0x2B;
volatile unsigned char *ddr_d = (unsigned char *)0x2A;
volatile unsigned char *pin_d = (unsigned char *)0x29;


//fan
volatile unsigned char *port_h = (unsigned char *)0x102;
volatile unsigned char *ddr_h = (unsigned char *)0x101;
volatile unsigned char *pin_h = (unsigned char *)0x100;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

volatile unsigned char *portADCDataRegisterHigh = (unsigned char *)0x79;

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;

volatile bool press = false;


enum state{
  Idle,
  Running,
  Disabled,
  Error,
};

enum state Current = Disabled;

void setup() {
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  
  U0init(9600);

  lcd.begin(16,2);
  lcd.setCursor(2,0);
  lcd.write((byte)2);


  adc_init();

  //water level setup

  attachInterrupt(digitalPinToInterrupt(interruptPin), power, RISING);
}

void loop() {

  DateTime time = rtc.now(); 

    if(Current == Disabled){

        disabled_state();
    }
    else if(Current == Idle){
        idle_state();
    }
    else if(Current == Error){
        error_state();
    }
    else if(Current\ == Running){
        running_state();
    }
}

  void idle_state(){
    /*In this state the fan would be turned off, and the Green LED would turn on, it should
    also display temp and humidity to the LCD; DISPLAY TO LCD HAS NOT BEEN IMPLEMENTED
    */

    fanOff(); 

    if(low()){
      setState(Error); 
    }

    *ddr_k |= (1 << IdleLed); // set LED_PIN as output
    *port_k |= (1 << IdleLed); // turn on LED_PIN

  }

  void disabled_state(){
    //In this state the Fan would be turned off and the Yellow LED would turn on 
    fanOff();

    // Set yellow LED pin as output and turn it on
    *ddr_h |= (1 << DisabledLed); // set LED_PIN as output
    *port_h |= (1 << DisabledLed); // turn on LED_PIN
    delay(1000); // wait for 1 second
  }

  void error_state(){
    bool errorLed = LOW;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Check water");
    writePort(PORTL, 3, errorLed);
  }

  void running_state(){
    /*In this state the fan would be turned on upon entry it should
    also display temp and humidity to the LCD; DISPLAY TO LCD HAS NOT BEEN IMPLEMENTED
    */
    fanOn();

    *ddr_e |= (1 << RunningLed); // set LED_PIN as output
    *port_e |= (1 << RunningLed); // turn on LED_PIN

  }


bool low(){
  if (adc_read(1) < WATER_THRESHOLD){
    return true;
  }
  return false;
}




void power(){
  U0putchar("POWER\n");
}

void reset(){
  U0putchar("RESET\n");
}

void fanOn(){
  // Set fan pin as output
  *ddr_h |= (1 << fanPIN); // set fanPIN as output

  // Turn off fan
  *port_h |= (1 << fanPIN); // turn on fanPIN

}

void fanOff(){
  *ddr_h |= (1 << fanPIN); // set fanPIN as output
  *port_h &= ~(1 << fanPIN); // turn off fanPIN
}




void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud -1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}


unsigned char U0kbhit()
{
  return UCSR0A & RDA;
}

void U0putchar(unsigned char U0pdata) { // Serial.Write
  while (!(*myUCSR0A & TBE));
  *myUDR0=U0pdata;
}