//ANKEET SINGH
//PARIS ACOSTA
//BRENDAN SMITH
//CPE 301 FINAL PROJECT

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

#define LCD_INTERVAL 60000
#define TEMP_THRESHOLD 20
#define WATER_THRESHOLD 100
#define DHT_PIN 7

#define WATER_SENSOR 5

#define RDA 0x80
#define TBE 0x20

#define power 22

int level = 0;

const int RS = 13, EN = 12, D4 = 35, D5 = 37, D6 = 39, D7 = 41;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

const byte interruptPin = 2;

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

// Pin Definitions
const int SPEED_PIN = 3;
const int DIR1 = 5;
const int DIR2 = 3;
const int powerButton = 18;
const int resetButton = 19;
const int fanPIN = 9;
const int IdleLed = 52;
const int DisabledLed = 50;
const int RunningLed = 48;
const int ErrorLed = 46;

// Enum for States
enum state {
Idle,
Running,
Disabled,
Error,
};

enum state State;

const int stepsPerRevolution = 2038;


// Global Variables
//enum state Current = Disabled;
//Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);
RTC_DS1307 rtc;
DHT dht(DHT_PIN, DHT11);

// Function to Write to a Port
void writePort(volatile unsigned char* reg, unsigned char bit, bool state) {
bit = 0x01 << bit;
if (state) {
*reg |= bit;
} else {
*reg &= ~bit;
}
}

void U0init(int U0baud) {
unsigned long FCPU = 16000000;
unsigned int tbaud;
tbaud = (FCPU / 16 / U0baud - 1);
*myUCSR0A = 0x20;
*myUCSR0B = 0x18;
*myUCSR0C = 0x06;
*myUBRR0 = tbaud;
}

void adc_init() {
// setup the A register
*my_ADCSRA |= 0b10000000; // set bit 7 to 1 to enable the ADC
*my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
*my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
*my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
// setup the B register
*my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
*my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
// setup the MUX Register
*my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
*my_ADMUX |= 0b01000000; // set bit 6 to 1 for AVCC analog reference
*my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
*my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

// Setup Function
void setup() {
rtc.begin();
rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

U0init(9600);

lcd.begin(16, 2); // Initialize the LCD with 16 columns and 2 rows

adc_init();
attachInterrupt(digitalPinToInterrupt(interruptPin), power, RISING);

dht.begin();

// Initialize State
State = Disabled;
}
// Loop Function
/////////////
void loop() {
// Read the water level from the sensor
int waterLevel = adc_read(WATER_SENSOR);

// Check if the water level is below the threshold
if (waterLevel < WATER_THRESHOLD) {
lcd.clear();
error_state();
}

if(waterLevel > WATER_THRESHOLD){
lcd.clear();
running_state();
}
// Handle the state transitions and actions
switch (State) {
case Disabled:
disabled_state();
break;
case Error:
error_state();
break;
case Running:
running_state();
break;
case Idle:
idle_state();
break;
}

// Display current time
displayTime();
}

void setState(enum state newState) {
switch(State) {
case Disabled:
if (newState == Idle) {
State = newState;
}
break;

case Error:
if (newState == Idle || newState == Disabled) {
State = newState;
}
break;

case Running:
State = newState;
break;

case Idle:
State = newState;
break;
}
}

// State Functions
void idle_state() {
fanOff();
if (low()) {
setState(Error);
}
displayTempAndHumidity();
digitalWrite(IdleLed,HIGH);
}

void disabled_state() {
fanOff();
lcd.print("Fan Off");
delay(1000);
digitalWrite(IdleLed,HIGH);
}

void error_state() {
lcd.clear(); // Clear the LCD screen
lcd.setCursor(0, 0); // Set cursor to the first row, first column
lcd.print("Error Check water");
digitalWrite(ErrorLed, HIGH); // Turn on Error LED
}

void running_state() {
fanOn();
displayTempAndHumidity();
digitalWrite(RunningLed,HIGH);
}

bool low() {
if (adc_read(1) < WATER_THRESHOLD){
return true;
}
return false;
}

void reset() {
U0putchar("RESET\n");
}

void fanOn() {
writePort(PORTH, fanPIN, HIGH);
}

void fanOff() {
writePort(PORTH, fanPIN, LOW);
}

void displayTempAndHumidity() {
float temperature = dht.readTemperature();
float humidity = dht.readHumidity();
lcd.setCursor(1, 0);
lcd.print("Temp: ");
lcd.print(temperature);
lcd.print("C");
lcd.setCursor(10, 1);
lcd.print("Hum: ");
lcd.print(humidity);
lcd.print("%");
}

void displayTime() {
DateTime time = rtc.now();
lcd.setCursor(0, 0);
lcd.print(time.hour());
lcd.print(":");
lcd.print(time.minute());
lcd.print(":");
lcd.print(time.second());

}



// ADC Functions


unsigned int adc_read(unsigned char adc_channel_num) {
// clear the channel selection bits (MUX 4:0)
*my_ADMUX &= 0b11100000;
// clear the channel selection bits (MUX 5)
*my_ADCSRB &= 0b11110111;
// set the channel number
if (adc_channel_num > 7) {
// set the channel selection bits, but remove the most significant bit (bit 3)
adc_channel_num -= 8;
// set MUX bit 5
*my_ADCSRB |= 0b00001000;
}
// set the channel selection bits
*my_ADMUX &= 0b11100000;
*my_ADMUX |= adc_channel_num;
// set bit 6 of ADCSRA to 1 to start a conversion
*my_ADCSRA |= 0x40;
// wait for the conversion to complete
while ((*my_ADCSRA & 0x40) != 0);
// return the result in the ADC data register
return *my_ADC_DATA;
}



unsigned char U0kbhit() {
return UCSR0A & RDA;
}

void U0putchar(unsigned char U0pdata) { // Serial.Write
while (!(*myUCSR0A & TBE));
*myUDR0 = U0pdata;
}