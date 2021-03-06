#ifndef com_file
#define com_file

#include <SPI.h>
#include <mcp2515.h>
#include <can.h>
#include "const.h"

//data for pack and unpack id from message
struct id_info {
  int8_t to;
  int8_t from;
  int8_t code;
};

//data from one message
struct message{
  struct id_info id;
  union my_can_msg value1;
  union my_can_msg value2;
};

//variable declaration
extern MCP2515 mcp2515; //SS pin 10

//notification flag for ISR and loop()
extern volatile bool interrupt;
extern volatile bool mcp2515_overflow;
extern volatile bool arduino_overflow;

//can frame class declaration
class can_frame_stream {
  can_frame cf_buffer[ buffsize ];
  int read_index; //where to read next message
  int write_index; //where to write next message
  bool write_lock; //buffer full

public:
  can_frame_stream() : read_index( 0 ) , write_index( 0 ), write_lock( false ) {};
  //int put( can_frame & );
  //int get( can_frame & );
  inline int put( can_frame &frame ){
    if( write_lock )
      return 0; //buffer full
    cf_buffer[ write_index ] = frame;
    write_index = ( ++write_index ) % buffsize;
    if( write_index == read_index)
      write_lock = true; //cannot write more
    return 1;
   }
   inline int get(can_frame &frame ){
    if( !write_lock && ( read_index== write_index ) )
      return 0; //empty buffer
    if( write_lock && ( read_index == write_index ) )
      write_lock = false; //release lock
    frame = cf_buffer[ read_index ];
    read_index = ( ++read_index ) % buffsize;
    return 1;
    }
  };
extern volatile can_frame_stream cf_stream; //create one object to use


//function declaration
MCP2515::ERROR write(uint32_t id, float val1, float val2, int m_len);
void irqHandler();
uint32_t  pack_id (struct id_info id_unp);
struct id_info unpack_id(uint32_t id_pack);
struct message read_message( bool* has_data);
void send_message( int8_t to,int8_t from, int8_t code, float val1, float val2);
bool check_acks( bool ack[], bool node[]);
short config_setup(float *slopes, bool* check_acks_flag, bool *acks, int* n_atual, int* estado, int n_max, bool node[],bool* done_acks);
#endif
