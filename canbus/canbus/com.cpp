#include "com.h"
#include "const.h"

//variable creation
volatile bool interrupt=false;
volatile bool mcp2515_overflow=false;
volatile bool arduino_overflow=false;
volatile can_frame_stream cf_stream; //create one object to use
MCP2515 mcp2515(10); //SS pin 10

//function that is called to handke the interrups and get data from canbus
void irqHandler(){
   can_frame frm;
  uint8_t irq = mcp2515.getInterrupts();
  //check messages in buffer 0
  if ( irq & MCP2515::CANINTF_RX0IF ) {
    mcp2515.readMessage( MCP2515::RXB0, & frm );
    if( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }
  //check messages in buffer 1
  if ( irq & MCP2515::CANINTF_RX1IF ) {
    mcp2515.readMessage( MCP2515::RXB1, & frm);
    if( !cf_stream.put( frm ) ) //no space
      arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if( (irq & MCP2515::EFLG_RX0OVR) |(irq & MCP2515::EFLG_RX1OVR) ) {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  interrupt = true; //notify loop()
} //end irqHandler()


//write funcion to canbus
MCP2515::ERROR write(uint32_t id, uint32_t val) {
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = message_len;
  my_can_msg msg;
  msg.value = val; //pack data
  for( int i = 0; i < frame.can_dlc; i++ ) //prepare can message
    frame.data[i] = msg.bytes[i];
  //send data
  return mcp2515.sendMessage(&frame);
}


//packs id
uint32_t  pack_id (struct id_info id_unp){
  uint32_t id=0;
  id |= id_unp.from;
  id <<=7;
  id |= id_unp.code;
  id <<= 2;
  id |= id_unp.to;
  return id;
}

//unpacks id
struct id_info unpack_id(uint32_t id_pack){
  struct id_info id;
  id.to = id_pack & 0x3;
  id_pack>>=2;
  id.code = id_pack & 0x7F;
  id_pack >>=7;
  id.from = id_pack & 0x3;  
  
  return id;
  }
  
