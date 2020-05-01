#ifndef MAESTRO_H
#define MAESTRO_H

#include <string>

namespace maestro
{

  // A very simple class to communicate w/ the Pololu Maestro board via USB.

  class Maestro
  {
  private:
    int fd_;

    bool writeBytes(const uint8_t *bytes, ssize_t size) const;

    bool readBytes(uint8_t *bytes, ssize_t size) const;

    bool getValue(uint8_t channel, uint16_t &value);

  public:
    Maestro();

    ~Maestro();

    bool connect(const std::string& port);

    void disconnect();

    bool ready() const;

    bool setPWM(uint8_t channel, uint16_t value);

    bool getPWM(uint8_t channel, uint16_t &value);

    bool getAnalog(uint8_t channel, double &value);

    bool getDigital(uint8_t channel, bool &value);
  };

} // namespace maestro

#endif // MAESTRO_H
