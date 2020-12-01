#ifndef constants_file
#define constants_file

//variables that can be changed
#define DEBUG
#define FILTERS
//#define LOOPBACK


static const int buffsize = 10; //Size of comunication arduino buffer
static const int message_len=8; //TO BE CHANGED
static const uint32_t mask = 0x00000003;
static const uint32_t filt1 = 0x00000000; //for filter the message for ID=0 - all nodes receive
//static const int filt2 = myID;//for filter messages for this specific node

//union to pack/unpack long ints into bytes
union my_can_msg {
  float value;
  unsigned char bytes[4]; //TO BE CHANGED
};

#endif
