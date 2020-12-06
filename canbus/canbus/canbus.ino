#include "const.h"
#include "com.h"
#include <SPI.h>
#include <mcp2515.h>
#include <EEPROM.h>

//to be moved
//unsigned long counter = 0;
//int myID=0;
short config_flag=0;
int myID=0;
bool ack [n_max];
bool nodes[n_max];
bool check_acks_flag=0;

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

  nodes[myID]=1;
  ack[myID]=1;

  if (myID==1){
    send_message( 0,myID, 0, 0, 0);
    config_flag=1;
    check_acks_flag=1;    
  }
}

void loop() {
  struct message m_read;
  bool has_data;
  unsigned int counter=0;
  bool done_acks=0;
  int state_config=0;
  int node_config=0;
  float slopes[n_max];    
  
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
      switch (m_read.id.code){
        //beguining of start up procedure messages
        case 0:
          if (myID==1){
            nodes[m_read.id.from]=1;
            config_flag=1;
          }
          else if (m_read.id.from==1){
            config_flag=1;
            // !!!!!!!! stops all streams of data !!!!!!!!
            // !!!!!!! put output at zero !!!!!!!
            send_message( m_read.id.from,myID, 1, 0, 0);
          }
          break;
        case 1:
          if(nodes[m_read.id.from]==0){
            nodes[m_read.id.from]=1;
          }
          ack[m_read.id.from]=1; //put one in the vector
          break;
        case 2:
          //!!!!!!!!!! get lux value !!!!!!!!
          //!!!!!!!!!! calculate the slope for number in message !!!!!!!!!!!
          send_message(1,myID,1,m_read.value1.value,2);
          break;
        case 3: 
          if(m_read.value1.value==0){
            //!!!!!!!!!!! turn light off !!!!!!!!!!
          }
          else if(m_read.value1.value==1){
            // !!!!!!!!!!!!!! turn light at max !!!!!!!!!!!!
          }
          send_message(1,myID,1,m_read.value1.value,3);
          break;
        case 4:
          config_flag=2;
          send_message( 1,myID,1,0,4);
          break;
        case 5:
          //not use 
          break;
        //end of start up procedure messages
        //beguining of hub fuction messages
        case 6:
          break;
        case 7:
          break;
        case 8:
          break;
        case 9:
          break;
        case 10:
          break;
        case 11:
          break;
        case 12:
          break;
        case 13:
          break;
        //end of hub fuction messages
        case 14:
          Serial.println("Code 14");
          break;
        
      }
      m_read = read_message( &has_data);
    }
  }

  //checking acks
  if (check_acks_flag==1){
    done_acks=check_acks(ack,nodes); 
    if (done_acks==1){
      check_acks_flag=0;
      //reset ack vector
      for(int i=0;i<n_max;i++){
        ack[i]=0;
      }
    }
  }

  if(done_acks==1 && config_flag==1){
    done_acks=0;
    config_flag=config_setup(slopes, &check_acks_flag, ack, &node_config, &state_config, n_max, nodes,&done_acks);
  }

  //indicates its presence to node 1
  if (myID!=1 && config_flag==0){
    if (counter<=1000){
      counter+=1;
    }
    else{
      send_message(1,myID,0,0,0);
      counter=0;
    }
  }
  delay(100); //some time to breath
}
