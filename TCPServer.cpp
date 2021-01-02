#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;

class server{
  ip::tcp::socket sock; // socket object
  streambuf input_buffer; //buffer to store the incoming server messages
  ip::tcp::acceptor acc; //to accept the socket connection
  char receive_msg[50];
  serial_port sp;
  steady_timer tim;
  char send_buffer[1024]; // buffer to store the data to send to server
  void start_connect_client(char *host, char *port);
  void start_read_write_client();
  public:
    server(io_context & io): sock {io}, acc{io, ip::tcp::endpoint{host, port}}, sp{io}, tim{io} { }
    void start(char *host, char *port) {
		start_connect_client(host, port);
		start_read_write_client();
    }
};

void server::start_connect_client(char *host, char *port){
  boost::system::error_code ec;
  sp.open("/dev/ttyACM0", ec); //connect to port
  if(ec) std::cout << "Could not open serial port \n";
  sp.set_option(serial_port_base::baud_rate {500000},ec);
  
  acc.async_accept(sock, [this](ec, const ip::tcp::endpoint &ep)
  {
	std::cout << "Connected to " << ep << std::endl;
    start_read_write_client();  
  });
}

void server::start_read_write_client()
{
  async_read_until(sock, input_buffer, '\n',
  [this](const boost::system::error_code & err, size_t sz )
  {
    if (!err)
    {
      std::string line;
      std::istream is {&input_buffer };
      std::getline(is, line);
      if (!line.empty())
        terminated_line = line + std::string("\n");
		std::size_t n = terminated_line.size();
		terminated_line.copy(send_buffer, n);
		async_write(sp, buffer(send_buffer, n), [this](const boost::system::error_code & err, size_t sz) {tim.expires_after(chrono::seconds{2});});
      }
      else
      {
        std::cout << "Error on receive: " << err.message() << "\n";
      }
    }
  );
  sp.async_read_some(buffer(receive_msg, 25), [this](const boost::system::error_code & err, size_t sz) {tim.expires_after(chrono::seconds{2});}
	
  async_write(sock, buffer(receive_msg, 25), [this](const boost::system::error_code & err, size_t sz) {std::cout << "\n" << std::endl;});
}

int main(int argc, char* argv[]) 
{
	if (argc != 3){
		std::cerr << "Usage: server <host> <port>\n";
		return 1;
	}
	
	io_context io;
	server ser {io};
	ser.start(argv[1], argv[2]);
	io.run();
	return 0;
} 