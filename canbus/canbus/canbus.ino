#include "const.h"
#include "com.h"
#include <SPI.h>
#include <mcp2515.h>

unsigned long counter = 0;


void setup(){
  Serial.begin(500000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

  //setting filters and mask to receive only messages with my ID and generic ID
/*  mcp2515.setConfigMode();
  mcp2515.setFilterMask(MCP2515::MASK0, 0, mask);
  mcp2515.setFilter(MCP2515::RXF0, 0, filt1);
  mcp2515.setFilterMask(MCP2515::MASK1, 0, mask);
  mcp2515.setFilter(MCP2515::RXF2, 0, filt2);
  */
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode();
}

void loop() {
//send a few msgs in a burst
  for( int i = 0; i < 4 ; i++ ) {
  Serial.print( "Sending: " );
  Serial.println( counter );
  if( write( i , counter++ ) != MCP2515::ERROR_OK )
    Serial.println( "\t \t \t \t MCP2515 TX Buf Full" );
  }
  delay(10000);
  if( interrupt ) {
    interrupt = false;
    if( mcp2515_overflow ) {
      Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
      mcp2515_overflow = false;
    }
    if( arduino_overflow ) {
      Serial.println( "\t\t\t\tArduino Buffers Overflow" );
      arduino_overflow = false;
    }
    can_frame frame;
    bool has_data;
    cli(); has_data = cf_stream.get( frame ); sei();
    while (has_data){
      my_can_msg msg;
      for( int i = 0 ; i < 4 ; i++ )
        msg.bytes[ i ] = frame.data[ i ];
      Serial.print( "\t \t Receiving: " );
      Serial.println( msg.value );
      cli(); has_data = cf_stream.get( frame ); sei();
    }
  }
  delay(1); //some time to breath
}
