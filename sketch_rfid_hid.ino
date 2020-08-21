#include "pins_arduino.h"
#include <SPI.h>
#include <Servo.h>



////////////////////////////////////////////////////////////////////////////////
// pin change interrupt handlers

// All I/O pins on the Atmega168 are covered by pin change interrupts. The PCINT
// corresponding to the pin must be enabled and masked and an ISR routine
// provided. Since PCINTs are per port, not per pin, the ISR must use some logic
// to actually implement a per-pin interrupt service.

// Pin to interrupt map:
// D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
// D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
// A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1

volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2,
};

static int pin_interrupt_mode[24];

typedef void (*interrupt_fn_t)(void);

volatile static interrupt_fn_t pin_interrupt_functions[24] = { 
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

volatile static uint8_t pin_register_last_state[3];

 void enable_pin_interrupt(uint8_t pin, void (*cb)(void), int mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  if (port == 1) {
     slot = port * 8 + (pin - 14);
  } else {
     slot = port * 8 + (pin % 8);
  }

  pin_interrupt_mode[slot] = mode;
  pin_interrupt_functions[slot] = cb;
  // set the mask
  *pcmask |= bit;
  // enable the interrupt for this port
  PCICR |= 0x01 << port;
}

void disable_pin_interrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  *pcmask &= ~bit;

  // disable the port interrupt if there are no pin interrupts remaining
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

static void port_interrupt_handler(uint8_t port) {
  // get the pin states for the indicated port.
  uint8_t curr = *portInputRegister(port + 2);
  uint8_t mask = curr ^ pin_register_last_state[port];
  pin_register_last_state[port] = curr;

  // mask is the pins that have changed; filter out those that don't have
  // registered handlers
  mask &= *port_to_pcmask[port];
  if (mask == 0) {
    return;
  }

  for (uint8_t x = 0; x < 8; x++) {
    uint8_t bit = 0x01 << x;
    if (!(bit & mask)) {
      continue;
    }

    uint8_t pin = port * 8 + x;
    if (pin_interrupt_functions[pin] == NULL) {
      continue;
    }

    if ((pin_interrupt_mode[pin] == CHANGE) ||
        ((pin_interrupt_mode[pin] == RISING) && (curr & bit)) ||
        ((pin_interrupt_mode[pin] == FALLING) && !(curr & bit))) {
      pin_interrupt_functions[pin]();
    }
  }
}

SIGNAL(PCINT0_vect) {
  port_interrupt_handler(0);
}
SIGNAL(PCINT1_vect) {
  port_interrupt_handler(1);
}
SIGNAL(PCINT2_vect) {
  port_interrupt_handler(2);
}



////////////////////////////////////////////////////////////////////////////////
// wiegand protocol decoder

volatile uint8_t wiegand_count;
volatile uint8_t wiegand_data[16];
volatile uint32_t wiegand_last_recv_time;

void on_wiegand_bit(uint8_t bit) {
  if (bit) {
    wiegand_data[wiegand_count >> 3] |= (1 << (7 - (wiegand_count & 7)));
  }
  wiegand_count++;
  wiegand_last_recv_time = millis();
}

void on_wiegand_zero() {
  on_wiegand_bit(0);
}

void on_wiegand_one() {
  on_wiegand_bit(1);
}

void clear_wiegand_value() {
  wiegand_count = 0;
  for (uint8_t x = 0; x < 16; x++) {
    wiegand_data[x] = 0;
  }
  wiegand_last_recv_time = millis();
}

void enable_wiegand_reader(int data0_pin, int data1_pin) {
  clear_wiegand_value();
  pinMode(data0_pin, INPUT);
  pinMode(data1_pin, INPUT);
  enable_pin_interrupt(data0_pin, on_wiegand_zero, FALLING);
  enable_pin_interrupt(data1_pin, on_wiegand_one, FALLING);
}



////////////////////////////////////////////////////////////////////////////////

uint8_t read_capacitive_pin(int pin_num) {
  volatile uint8_t* port = portOutputRegister(digitalPinToPort(pin_num));
  volatile uint8_t* ddr = portModeRegister(digitalPinToPort(pin_num));
  byte bitmask = digitalPinToBitMask(pin_num);
  volatile uint8_t* pin = portInputRegister(digitalPinToPort(pin_num));

  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);

  // Prevent the timer IRQ from disturbing our measurement
  uint8_t SREG_old = SREG; //back up the AVR Status Register
  noInterrupts();

  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
  SREG = SREG_old;

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}



////////////////////////////////////////////////////////////////////////////////
// main program

// pin assignments
int reader_data0_pin = 3; // green from R40
int reader_data1_pin = 2; // white from R40
int servo_control_pin = A5; // white wire from servo
int relay_control_pin = A1;
int reader_hold_pin = 4; // blue from R40
int reader_beeper_pin = 5; // yellow from R40
int reader_green_led_pin = 6; // orange from R40
int reader_red_led_pin = 7; // brown from R40
int exit_sense_pin = A0; // exit switch
int ir_phototransistor_sense_pin = A4;

Servo servo;

void setup() {
  enable_wiegand_reader(reader_data0_pin, reader_data1_pin);
  pinMode(relay_control_pin, OUTPUT);
  pinMode(reader_hold_pin, OUTPUT);
  pinMode(reader_beeper_pin, OUTPUT);
  pinMode(reader_green_led_pin, OUTPUT);
  pinMode(reader_red_led_pin, OUTPUT);
  pinMode(ir_phototransistor_sense_pin, INPUT);
  delay(100);

  Serial.begin(115200);
  Serial.println("inited");

  digitalWrite(reader_hold_pin, HIGH); // disable HOLD
  digitalWrite(reader_beeper_pin, HIGH); // disable BEEPER
  digitalWrite(reader_green_led_pin, HIGH); // disable GREEN LED
  digitalWrite(reader_red_led_pin, HIGH); // disable RED LED

  servo.attach(servo_control_pin);

  actuate_servo_temporary(0);
}

void actuate_servo_temporary(int position) {
  servo.write(position);

  digitalWrite(relay_control_pin, HIGH); // activate relay (activating servo)
  delay(1500); // wait for servo to get to position
  digitalWrite(relay_control_pin, LOW); // deactivate relay + servo
}

bool is_open() {
  return digitalRead(ir_phototransistor_sense_pin) == LOW;
}

void unlock() {
  digitalWrite(reader_hold_pin, LOW); // enable HOLD
  digitalWrite(reader_green_led_pin, LOW); // set reader LED to green

  actuate_servo_temporary(180);
  long open_time = millis();
  delay(3000);
  long is_open_time = millis();
  while ((millis() - open_time < 10000) && (millis() - is_open_time < 1000)) {
    if (is_open()) {
      is_open_time = millis();
    }
    delay(100);
  }
  actuate_servo_temporary(0);

  digitalWrite(reader_green_led_pin, HIGH); // set reader LED to red
  digitalWrite(reader_hold_pin, HIGH); // disable HOLD
}

bool memcmp_bits(const uint8_t* a, const uint8_t* b, size_t bits) {
  while (bits >= 8) {
    if (*a != *b) {
      return true;
    }
    a++;
    b++;
    bits -= 8;
  }

  uint8_t mask_for_bit_count[8] = {
    0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE,
  };
  if (bits != 0) {
    uint8_t mask = mask_for_bit_count[bits];
    if ((*a & mask) != (*b & mask)) {
      return true;
    }
  }

  return false;
}

void on_card_read(const uint8_t* data, uint16_t bit_count) {
  Serial.print("on_card_read: bits=");
  Serial.print(bit_count);
  Serial.print(" data=0x");
  uint8_t bytes_to_print = bit_count >> 3;
  if ((bytes_to_print << 3) != bit_count) {
    bytes_to_print++;
  }
  for (uint16_t offset = 0; offset < bytes_to_print; offset++) {
    // Serial.print(..., HEX) doesn't print leading zeroes
    if ((data[offset] & 0xF0) == 0x00) {
      Serial.print("0");
    }
    Serial.print(data[offset], HEX);
  }
  Serial.println();

  // TODO: Add your cards here! Set num_accepted appropriately, then remove the
  // examples and put your own card's ID. To find your card's ID, wire up the
  // reader and power the Arduino from your computer's USB port. The Arduino
  // will print the card's ID to the serial monitor each time the reader reads
  // a card. But watch out: after power-on, sometimes the first card read by
  // the reader is sent incorrectly. To make sure you have the right ID, read
  // it twice.
  const uint8_t num_accepted = 2;
  const uint8_t expected_data[num_accepted][16] = {
    // 35-bit example
    {0xA5, 0x39, 0x58, 0x03, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // 42-bit example
    {0x38, 0x77, 0x2A, 0x94, 0xF6, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
  };
  const uint8_t expected_bits[num_accepted] = {
    35, 42,
  };

  bool found = false;
  for (uint8_t index = 0; index < num_accepted; index++) {
    if (bit_count != expected_bits[index]) {
      continue;
    }
    if (memcmp_bits(data, expected_data[index], bit_count)) {
      continue;
    }
    found = true;
  }

  if (found) {
    // if the door is already open, warn (by beeping twice)
    if (is_open()) {
      digitalWrite(reader_beeper_pin, LOW);
      delay(100);
      digitalWrite(reader_beeper_pin, HIGH);
      delay(100);
      digitalWrite(reader_beeper_pin, LOW);
      delay(100);
      digitalWrite(reader_beeper_pin, HIGH);
    }

    unlock();
  }
}

void loop() {
  wiegand_last_recv_time = millis();
  long last_open_sense_time = wiegand_last_recv_time;

  // wait until there's data, and we haven't received any new data for 100ms
  while ((wiegand_count == 0) || (wiegand_last_recv_time > millis() - 100)) {
    if (last_open_sense_time < millis() - 500) {
      last_open_sense_time = millis();
      is_open();
    }
    if (digitalRead(exit_sense_pin) == HIGH) {
      unlock();
    }
  }

  if (wiegand_count > 0) {
    on_card_read(wiegand_data, wiegand_count);
  }

  clear_wiegand_value();
}
