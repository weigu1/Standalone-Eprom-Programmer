/*
  eprom_programmer_27c512_mega32  
  Atmega644p

  ---------------------------------------------------------------------------
  Copyright (C) 2023 Guy WEILER www.weigu.lu

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  ---------------------------------------------------------------------------
*/

//#define DEBUG

#include "extEEPROM.h"                    // Tools -> Manage Libraries... inst.

extEEPROM eep(kbits_512, 1, 128, 0x50);   //create EEPROM object adr = 0x50

const unsigned long EPROM_SIZE_BYTE = 65536;
const unsigned int PAGE_LENGTH_BYTE = 128;
unsigned int NR_OF_PAGES = EPROM_SIZE_BYTE/PAGE_LENGTH_BYTE;
byte page_data[PAGE_LENGTH_BYTE];

const byte PIN_PUSHBUTTON = 10; // PD2
const byte PIN_GET_SWITCH = 11; // PD3
const byte PIN_EPROM_E = 14; // EPROM Enable PD6
const byte PIN_LED = 15; // PD7

byte eprom_data_byte = 0;

/****** SETUP ******/
void setup() {
  Serial.begin(38400); // 38400 more accurate than 1115200!
  while (!Serial);
  #ifdef DEBUG
    Serial.println("\nHello from your 27C512 Programmer");
  #endif // DEBUG  
  eep.begin(eep.twiClock400kHz);          // init I2C bus (use 400kHz) 
  // Address ports 16 bit (PortA and PortB)
  DDRA = 0xFF; // adressing with PortA and B (OUTPUT)
  DDRB = 0xFF; 
  
  pinMode(PIN_PUSHBUTTON, INPUT_PULLUP);
  pinMode(PIN_GET_SWITCH, INPUT);
  pinMode(PIN_EPROM_E, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_EPROM_E,HIGH);  // disable Eprom
  digitalWrite(PIN_LED,HIGH);      // switch LED off (neg. logic)
} 

/****** MAINLOOP ******/
void loop() {
  if (!digitalRead(PIN_PUSHBUTTON)) { // if yellow button pressed
    #ifdef DEBUG
      Serial.print("Bushbutton pressed: ");
    #endif // DEBUG  
    if (digitalRead(PIN_GET_SWITCH)) {  
      #ifdef DEBUG
        Serial.print("Programming Mode");  // Programming Mode
      #endif // DEBUG  
      write_eprom_from_eeprom();  
    }
    else {
      #ifdef DEBUG
        Serial.print("Reading Mode");      // Reading Mode
      #endif // DEBUG  
      //choose one of the following:      
      //read_eprom_2_serial_bytestream();
      //copy_eprom_2_eeprom_and_serial_output();
      copy_eprom_2_eeprom();
      //read_eeprom_2_serial_bytestream();
      //compare_eprom_2_eeprom();
      //read_eprom_2_serial_ascii_cooked();
      //read_eprom_2_serial_ascii();
    }  
  }  
  delay(300);
}

/****** EPROM and EEPROM Functions ******/

void eprom_set_address(unsigned int adr) {
  PORTA = lowByte(adr);  
  PORTB = highByte(adr);  
  delayMicroseconds(1);
}

void data_port_input() {
  // Data port 8 bit (6 bit on PortC and 2 bit on PortD)
  DDRC = DDRC & 0x03; // PortC (without SCL/SDA PC0,PC1)
  DDRD = DDRD & 0xCF; // 2 datalines on PortD (PD4 and PD5)
  delayMicroseconds(1);
}  

void data_port_output() {
  // Data port 8 bit (6 bit on PortC and 2 bit on PortD)
  DDRC = DDRC | 0xFC; // PortC (without SCL/SDA PC0,PC1)
  DDRD = DDRD | 0x30; // 2 datalines on PortD (PD4 and PD5)
  delayMicroseconds(1);
}  

void read_eprom_setup() {
  digitalWrite(PIN_LED,LOW);      // switch LED on
  digitalWrite(PIN_EPROM_E,LOW);  // enable EPROM
  data_port_input();  
  #ifdef DEBUG
    Serial.print("Reading Eprom... ");  
  #endif // DEBUG
}

void read_eprom_setdown() {  
  digitalWrite(PIN_EPROM_E,HIGH);  // disable EPROM
  #ifdef DEBUG
    Serial.println("done");  
  #endif // DEBUG
  digitalWrite(PIN_LED,HIGH);      // switch LED off
}

void read_eprom_2_serial_bytestream() {
  read_eprom_setup();
  for (unsigned long adr = 0 ; adr < EPROM_SIZE_BYTE; adr++) {    
    eprom_set_address(adr);
    eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
    Serial.write(eprom_data_byte);    
  }
  read_eprom_setdown();
}

void copy_eprom_2_eeprom() {
  unsigned long adr = 0;
  read_eprom_setup();
    for (unsigned int page = 0; page < NR_OF_PAGES; page++) {
      for (unsigned int byte_in_page = 0; byte_in_page < PAGE_LENGTH_BYTE; byte_in_page++) {     
        adr = byte_in_page + page*PAGE_LENGTH_BYTE;      
        eprom_set_address(adr);
        eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
        page_data[byte_in_page] = eprom_data_byte;        
      }    
    eep.write(page*PAGE_LENGTH_BYTE, page_data, PAGE_LENGTH_BYTE); // write page                  
  }    
  read_eprom_setdown();
}

void read_eeprom_2_serial_bytestream() {  
  digitalWrite(PIN_LED,LOW);      // switch LED on
  for (unsigned int page = 0 ; page < NR_OF_PAGES; page++) {    
    eep.read(page*PAGE_LENGTH_BYTE, page_data, PAGE_LENGTH_BYTE); //    
    for (unsigned int byte_in_page = 0 ; byte_in_page < PAGE_LENGTH_BYTE; byte_in_page++) {    
      Serial.write(page_data[byte_in_page]);      
    }    
  }    
  digitalWrite(PIN_LED,HIGH);      // switch LED off
}

void compare_eprom_2_eeprom() {
  unsigned long adr = 0;
  read_eprom_setup();
  for (unsigned int page = 0 ; page < NR_OF_PAGES; page++) {    
    eep.read(page*PAGE_LENGTH_BYTE, page_data, PAGE_LENGTH_BYTE); //    
    for (unsigned int byte_in_page = 0 ; byte_in_page < PAGE_LENGTH_BYTE; byte_in_page++) {    
      adr = byte_in_page + page*PAGE_LENGTH_BYTE;      
      eprom_set_address(adr);      
      eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
      if (page_data[byte_in_page] != eprom_data_byte) {
        Serial.println("Error while checking EEPROM content!");
        digitalWrite(PIN_LED,HIGH);      // switch LED off
        return;
      }
    }     
  }
  read_eprom_setdown();
  Serial.println("Everything OK :)");
}

void write_eprom_from_eeprom() {  
  unsigned long adr = 0;
  digitalWrite(PIN_LED,LOW);        // switch LED on
  digitalWrite(PIN_EPROM_E, HIGH);  //disable EPROM
  data_port_output();  
  #ifdef DEBUG
    Serial.print("Writing Eprom... ");  
  #endif // DEBUG
  for (unsigned int page = 0 ; page < NR_OF_PAGES; page++) {    
    eep.read(page*PAGE_LENGTH_BYTE, page_data, PAGE_LENGTH_BYTE); //    
    for (unsigned int byte_in_page = 0 ; byte_in_page < PAGE_LENGTH_BYTE; byte_in_page++) {    
      adr = byte_in_page + page*PAGE_LENGTH_BYTE;      
      eprom_set_address(adr);      
      #ifdef DEBUG
        Serial.println(adr);  
      #endif // DEBUG
      PORTC = PORTC | (page_data[byte_in_page] & 0xFC);
      PORTC = PORTC & (page_data[byte_in_page] | 0x03);          
      PORTD =  PORTD | ((page_data[byte_in_page]<<4) & 0x30);
      PORTD =  PORTD & ((page_data[byte_in_page]<<4) | 0xCF);
      delayMicroseconds(3);
      digitalWrite(PIN_EPROM_E,LOW);  // programming pulse
      delayMicroseconds(100); 
      digitalWrite(PIN_EPROM_E,HIGH);  
      delayMicroseconds(3);
    }     
  }
  #ifdef DEBUG
    Serial.println("done");
  #endif // DEBUG
  digitalWrite(PIN_LED,HIGH);        // switch LED on
}

/****** Helper Functions ******/

void read_eprom_2_serial_ascii_cooked() {
  read_eprom_setup();
  for (unsigned long adr = 0 ; adr < EPROM_SIZE_BYTE; adr++) {    
    eprom_set_address(adr);     
    eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
    Serial.print("0x");      
    if (eprom_data_byte < 16) Serial.print("0");
    Serial.print(String(eprom_data_byte,HEX) + " ");
    if (adr%25 == 0) {
      Serial.println();
    }
  } 
  Serial.println(); 
  read_eprom_setdown();
}

void read_eprom_2_serial_ascii() {
  read_eprom_setup();
  for (unsigned long adr = 0 ; adr < EPROM_SIZE_BYTE; adr++) {    
    eprom_set_address(adr);    
    eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
    if (eprom_data_byte < 16) Serial.print("0");       
    Serial.print(eprom_data_byte,HEX);      
  }   
  Serial.println(); 
  read_eprom_setdown();
}

void copy_eprom_2_eeprom_and_serial_output() {
  unsigned long adr = 0;
  read_eprom_setup();
    for (unsigned int page = 0; page < NR_OF_PAGES; page++) {
      for (unsigned int byte_in_page = 0; byte_in_page < PAGE_LENGTH_BYTE; byte_in_page++) {     
        adr = byte_in_page + page*PAGE_LENGTH_BYTE;      
        eprom_set_address(adr);        
        eprom_data_byte = (PINC & 0xFC) | ((PIND & 0x30)>>4);
        page_data[byte_in_page] = eprom_data_byte;
        Serial.write(eprom_data_byte);    
      }    
    eep.write(page*PAGE_LENGTH_BYTE, page_data, PAGE_LENGTH_BYTE); // write page                  
  }    
  read_eprom_setdown();
}
