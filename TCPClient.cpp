#include <unistd.h>
#include <iostream>
#include <boost/asio.hpp>
using namespace boost::asio;

class client {
  ip::tcp::resolver res; // to find out the server coordinates
  ip::tcp::socket sock; // socket object
  streambuf input_buffer; //buffer to store the incoming server messages
  char receive_msg[50];
  posix::stream_descriptor input; // console object
  serial_port sp;
  steady_timer tim;
  streambuf console_buffer; // buffer to store the console input
  char send_buffer[1024]; // buffer to store the data to send to server
  void start_connect_server( char *host, char *port );
  void start_read_server() ;
  void start_read_console();
  public:
    client(io_context & io) : res {io}, sock {io}, input {io, ::dup(STDIN_FILENO)}, sp {io}, tim {io}, console_buffer {100000} { }
    void start( char *host, char *port) {
    start_connect_server(host, port);
    start_read_console();
    }
};

void client::start_connect_server(char *host, char *port) {
  boost::system::error_code ec;
  sp.open("/dev/ttyACM0", ec); //connect to port
  if(ec) std::cout << "Could not open serial port \n";
  sp.set_option( serial_port_base::baud_rate {500000},ec);

  res.async_resolve(ip::tcp::resolver::query(host, port),
    [this](const boost::system::error_code & err, ip::tcp::resolver::results_type results)
    {
      async_connect(sock, results,
        [this] (const boost::system::error_code & err, const ip::tcp::endpoint &ep)
        {
          std::cout << "Connected to " << ep << std::endl;
          start_read_server();
        }
      );
    }
  );
}

void client::start_read_server()
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
        std::cout << line << "\n";
        start_read_server();
      }
      else
      {
        std::cout << "Error on receive: " << err.message() << "\n";
      }
    }
  );
}

void client::start_read_console()
{
  async_read_until(input, console_buffer, '\n',
  [this](const boost::system::error_code & err, size_t sz) {
  if (!err) {
    std::string line, terminated_line;
    std::istream is( &console_buffer );
    std::getline(is, line);
    if (!line.empty()) { // Empty messages are ignored.
      terminated_line = line + std::string("\n");
      std::size_t n = terminated_line.size();
      terminated_line.copy( send_buffer, n) ;
      async_write(sock, buffer( send_buffer, n ), [this](const boost::system::error_code & err, size_t sz) { std::cout << "\n" << std::endl;});
        }
        start_read_console();
    }
      else {
        std::cout << "Error on read console handler: " << err.message() << "\n";
      }
    }
  );
}

int main(int argc, char* argv[]) {
  if (argc != 3){
    std::cerr << "Usage: client <host> <port>\n";
    return 1;
  }
  
  io_context io;
  client cli {io};
  cli.start( argv[1], argv[2] );
  io.run();
  return 0;
}
