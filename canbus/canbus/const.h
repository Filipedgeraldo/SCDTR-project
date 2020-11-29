#ifndef constants_file
#define constants_file

const int myID=2; //id of the node

static const int buffsize = 10; //Size of comunication arduino buffer
static const int message_len=4;
static const uint32_t mask = 0x00000003;
static const uint32_t filt1 = 0x00000000; //for filter the message for ID=0 - all nodes receive
static const int filt2 = myID;//0x00000003;  //for filter messages for this specific node

//union to pack/unpack long ints into bytes
union my_can_msg {
  unsigned long value;
  unsigned char bytes[4];
};

union id_analysis {
  uint32_t id_value;
  unsigned char bits[32];
};

#endif
