#include "const.h"
#include "com.h"
#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

//to be moved
unsigned long counter = 0;
int myID=0;

void setup(){
  Serial.begin(500000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  //getting id from eeprom
  myID=EEPROM.read(0);
  #ifdef DEBUG
    Serial.print("myId: ");
    Serial.println(myID);
  #endif
  //setting filters and mask to receive only messages with my ID and generic ID
  #ifdef FILTERS
  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilter(MCP2515::RXF0, 0, filt1);
  mcp2515.setFilter(MCP2515::RXF1, 0, myID);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  mcp2515.setFilter(MCP2515::RXF2, 0, myID);
  mcp2515.setFilter(MCP2515::RXF3, 0, filt1);
  #endif

  #ifndef LOOPBACK
  mcp2515.setNormalMode();
  #endif
  #ifdef LOOPBACK
  mcp2515.setLoopbackMode();
  #endif
}

void loop() {
  struct id_info id_s;
  struct message m_read;
  bool has_data;
  
  //defines id for message - just for testing porpouse
  id_s.from=myID;
  id_s.to =2;
  id_s.code= 10;

  for( int i = 0; i < 4 ; i++ ) {
  Serial.print( "Sending: " );
  Serial.println( counter );
  id_s.to =i;
  if( write( pack_id(id_s) ,counter++, counter,8) != MCP2515::ERROR_OK )
    #ifdef DEBUG
      Serial.println( "\t \t \t \t MCP2515 TX Buf Full" );
    #endif
  }
  
  
  if( interrupt ) {
    m_read = read_message( &has_data);
    while (has_data){
      #ifdef DEBUG
        Serial.print( "\t \t Receiving: " );
        Serial.println( m_read.value1.value );
        Serial.print( "\t \t Receiving: " );
        Serial.println( m_read.value2.value );
        Serial.print( "\t \t to: " );
        Serial.print(m_read.id.to);
        Serial.print( "\t \t from: " );
        Serial.print(m_read.id.from);
        Serial.print( "\t \t code: " );
        Serial.println(m_read.id.code);
      #endif
      m_read = read_message( &has_data);
    }
  }
  delay(100); //some time to breath
}
