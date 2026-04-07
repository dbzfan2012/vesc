// POSIX serial port shim — drop-in replacement for the serial::Serial API
// used by vesc_interface.cpp. Backed by termios/read/write.

#ifndef VESC_DRIVER_SERIAL_PORT_H_
#define VESC_DRIVER_SERIAL_PORT_H_

#include <string>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <cstddef>

namespace serial
{

// Enums matching the original serial library interface
enum bytesize_t { eightbits = 8 };
enum parity_t   { parity_none = 0 };
enum stopbits_t { stopbits_one = 1 };
enum flowcontrol_t { flowcontrol_none = 0 };

struct Timeout
{
  uint32_t read_timeout_ms;
  static Timeout simpleTimeout(uint32_t timeout_ms)
  {
    Timeout t;
    t.read_timeout_ms = timeout_ms;
    return t;
  }
};

class Serial
{
public:
  Serial(const std::string& port = "",
         uint32_t baudrate = 115200,
         Timeout timeout = Timeout::simpleTimeout(100),
         bytesize_t bytesize = eightbits,
         parity_t parity = parity_none,
         stopbits_t stopbits = stopbits_one,
         flowcontrol_t flowcontrol = flowcontrol_none);

  ~Serial();

  void setPort(const std::string& port);
  void setBaudrate(uint32_t baudrate);
  void setTimeout(Timeout timeout);

  void open();
  void close();
  bool isOpen() const;

  /// Returns number of bytes available to read
  size_t available();

  /// Read up to size bytes into buffer vector (appends to vector).
  /// Returns number of bytes actually read.
  size_t read(std::vector<uint8_t>& buffer, size_t size);

  /// Write buffer. Returns number of bytes written.
  size_t write(const std::vector<uint8_t>& data);

private:
  std::string port_;
  uint32_t baudrate_;
  Timeout timeout_;
  int fd_;
};

} // namespace serial

#endif // VESC_DRIVER_SERIAL_PORT_H_
