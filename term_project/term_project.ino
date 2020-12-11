#include <dht_nonblocking.h>
#include <LiquidCrystal.h>
#include <DS3231.h>
#include <Wire.h>
#include "Stepper.h"

#define DHT_PIN 51

DHT_nonblocking dht_sensor(DHT_PIN, DHT_TYPE_11);

LiquidCrystal lcd(49, 48, 45, 44, 43, 42);

Stepper vent(32, 22, 24, 23, 25);

DS3231 clock;
RTCDateTime dt;

volatile boolean test = false;

boolean is_off;
volatile boolean fan_on;
volatile boolean error;
volatile boolean is_hot;
volatile boolean turned;
volatile boolean clockwise;
volatile float temp;
volatile float humidity;
volatile unsigned int water_level;
volatile unsigned int ticks;
int rot_pos;
int prev_pos;
int num_steps;

volatile unsigned char* pin_a = (unsigned char*) 0x20;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* port_a = (unsigned char*) 0x22;

volatile unsigned char* pin_b = (unsigned char*) 0x23;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* port_b = (unsigned char*) 0x25;

volatile unsigned char* pin_d = (unsigned char*) 0x29;
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* port_d = (unsigned char*) 0x2B;

volatile unsigned char* pin_k = (unsigned char*) 0x106;
volatile unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* port_k = (unsigned char*) 0x108;

volatile unsigned char* pin_l = (unsigned char*) 0x109;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* port_l = (unsigned char*) 0x10B;

volatile unsigned char* adcsr_a = (unsigned char*) 0x7A;
volatile unsigned char* adcsr_b = (unsigned char*) 0x7B;
volatile unsigned char* ad_mux = (unsigned char*) 0x7C;
volatile unsigned int* adc_data = (unsigned int*) 0x78;

volatile unsigned char* eicr_a = (unsigned char*) 0x69;
volatile unsigned char* eimsk = (unsigned char*) 0x3D;
volatile unsigned char* eifr = (unsigned char*) 0x3C;

volatile unsigned char* timsk_1 = (unsigned char*) 0x6F;
volatile unsigned char* tccr_1a = (unsigned char*) 0x80;
volatile unsigned char* tccr_1b = (unsigned char*) 0x81;
volatile unsigned int* tcnt_1 = (unsigned int*) 0x84;

////////// SETUP //////////

void setup() {
  Serial.begin(9600);
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);
  // Set global interrupt flag
  sei();
  
  // Set INT2 interrupt flags
  *eimsk |= 0b00000100;
  *eicr_a |= 0b00100000;
  *eicr_a &= 0b11101111;

  // Initialize ADC
  adc_init();

  // LCD set-up
  lcd.begin(16, 2);

  // Set LED pins to output
  *ddr_b |= 0b11110000;

  // Set temp sensor and button pins to input
  *ddr_b &= 0b11111010;

  *ddr_d &= 0b11111000; // Set PD2:0 as input

  *ddr_k &= 0b01111111; // Set PK7 as input
  *port_k &= 0b01111111; 

  *port_b |= 0b00000001; // PB0 pull-up resistor
  
  *port_b &= 0b11111101; // Initialize temp sensor input

  *ddr_b |= 0b00001000; // Set PB3 as output

  *ddr_a &= 0b10011111; // Set PA4:5 as input
  *port_a |= 0b01000000; // PA5 pull-up resistor
  
  //*ddr_a |= 0b00001111; 

  // Activate yellow LED
  led_act('Y');

  // Default values
  is_off = true;
  fan_on = false;
  error = false;
  is_hot = false;
  ticks = 65500;
  water_level = 200;
}

////////// LOOP //////////

void loop() {
  vent.setSpeed(700);
  
  // If button is pushed
  if(*pin_b & 0b00000001) {
    if(is_off) {
      // Activate green LED
      led_act('G');
    } else {
      // Activate yellow LED
      led_act('Y');
      if(fan_on) {
        stop_fan();
      }
      lcd.clear();
    }
  
    while(*pin_b & 0b00000001); // Debouncing
    is_off = !is_off;
  }

  water_level = adc_read(15);
  
  // If AC is 'on'
  if(!is_off) {
    
    if(error) {
      led_act('R');
      if(fan_on) {
        stop_fan();
      }
      print_error();
      error = !error;
      while(adc_read(15) < 200);
    }

    if(measure_environment(&temp, &humidity)) {
      print_data();
    }

    if(is_hot) {
      led_act('B');
      if(!fan_on) {
        start_fan();
      }
    } else {
      led_act('G');
      if(fan_on) {
        stop_fan();
      }
    }

    if(!(*pin_a & 0b01000000)) {
      if(rot_pos != 0) {
        vent.step(-(rot_pos * 50));
        rot_pos = 0; // Reset position
      }
    }

    if(turned) {
      prev_pos = rot_pos;
      if(clockwise) {
        rot_pos = rot_pos - 1;
      } else {
        rot_pos = rot_pos + 1;
      }

      turned = !turned;

      if(rot_pos == (prev_pos + 1)) {
        num_steps = 50;
        vent.step(num_steps);
      } else if(rot_pos == (prev_pos - 1)) {
        num_steps = -50;
        vent.step(num_steps);
      }
    }

    *port_a &= 0b11110000;
  }
}

// Function for taking measurements from DHT11 sensor
static bool measure_environment(float* temp, float* humidity) {
  static unsigned long measurement_timestamp = millis();

  if(millis() - measurement_timestamp > 1000ul) {
    if(dht_sensor.measure(temp, humidity)) {
      measurement_timestamp = millis();
      return true;
    }
  }

  return false;
}

////////// ADC //////////

// Initialize ADC
void adc_init() {
  *adcsr_a |= 0b10001000;
  *adcsr_b &= 0b11010000;

  *adcsr_b &= 0b11110000;

  *ad_mux |= 0b01000000;
  *ad_mux &= 0b01000000;
}

// Read ADC
unsigned int adc_read(unsigned char adc_channel_num) {
  *ad_mux &= 0b11100000;
  *adcsr_b &= 0b11110111;

  if(adc_channel_num > 7) {
    adc_channel_num -= 8;
    *adcsr_b |= 0b00001000;
  }

  *ad_mux += adc_channel_num;
  *adcsr_a |= 0b01000000;

  while((*adcsr_a & 0b01000000) != 0);

  return *adc_data;
}

////////// LED //////////

// Activate LED of specified color
bool led_act(unsigned char color) {
  switch(color) {
    case 'R':
      *port_b &= 0b00011111;
      *port_b |= 0b00010000;
      return true;
    case 'G':
      *port_b &= 0b01001111;
      *port_b |= 0b01000000;
      return true;
    case 'B':
      *port_b &= 0b00101111;
      *port_b |= 0b00100000;
      return true;
    case 'Y':
      *port_b &= 0b10001111;
      *port_b |= 0b10000000;
      return true;
  }
  return false;
}

////////// ISR //////////

ISR(ADC_vect) {
  if(water_level < 200) {
    error = true;
  } else if(temp < 25) {
    error = false;
    is_hot = false;
  } else {
    error = false;
    is_hot = true;
  }
}

ISR(INT2_vect) {
  for(long i = 0; i < 10000; i++) // Debouncing
  if(*pin_d & 0b00000100) {
    test = true;
    clockwise = *pin_a & 0b00100000;
  } else {
    clockwise = !(*pin_a & 0b00100000);
  }
  turned = true;
}

ISR(TIMER1_OVF_vect) {
  *tccr_1b &= 0b11111000;
  *tccr_1a &= 0b11111100;

  *tcnt_1 = ticks;
  
  *tccr_1b &= 0b11100101;
  *tccr_1b |= 0b00000101;

  if(*tcnt_1 == ticks) {
    *port_b ^= 0b00001000;
  }
}


////////// PRINT //////////

void print_data() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temp);
  lcd.print(" C.");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
}

void print_error() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Error:");
  lcd.setCursor(0,1);
  lcd.print("Water level low."); 
}

void print_timestamp() {
  dt = clock.getDateTime();
  Serial.print(dt.year);
  Serial.print("-");
  Serial.print(dt.month);
  Serial.print("-");
  Serial.print(dt.day);
  Serial.print(" ");
  Serial.print(dt.hour);
  Serial.print(":");
  Serial.print(dt.minute);
  Serial.print(":");
  Serial.println(dt.second);
}

////////// TIMERS //////////

void start_fan() {
  print_timestamp();
  *timsk_1 |= 0b00000001;
  *tccr_1a &= 0b11111100;
  *tccr_1b &= 0b11100101;
  *tccr_1b |= 0b00000101;
  fan_on = !fan_on;
}

void stop_fan() {
  print_timestamp();
  *tccr_1b &= 0b11111000;
  *tccr_1a &= 0b11111100;
  *port_b &= 0b11110111;
  fan_on = !fan_on;
}
