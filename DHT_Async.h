#ifndef _DHT_ASYNC_H_
#define _DHT_ASYNC_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#define DHT_TYPE_11  0
#define DHT_TYPE_12  1
#define DHT_TYPE_21  2
#define DHT_TYPE_22  3


class DHT_Async {
public:
    DHT_Async(uint8_t pin, uint8_t type);

    bool measure(float *temperature, float *humidity);

    void wait();

    float convertCtoF(float c);

    float convertFtoC(float f);

private:
    uint8_t dhtState;
    unsigned long dhtTimestamp;
    uint8_t data[5];
    const uint8_t _pin, _type;
#ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and
    // bitmask for the digital pin connected to the DHT.  Other platforms will use
    // digitalRead.
    const uint8_t _bit, _port;
#endif
    const uint32_t _maxCycles;

    bool readData();

    bool readAsync();

    float readTemperature() const;

    float readHumidity() const;

    uint32_t expectPulse(bool level) const;
};


class DHT_Async_Interrupt {
public:
    DHT_Async_Interrupt() {
#if !defined(ARDUINO_ARCH_NRF52)
        noInterrupts();
#endif
    }

    ~DHT_Async_Interrupt() {
#if !defined(ARDUINO_ARCH_NRF52)
        interrupts();
#endif
    }
};

#endif //_DHT_ASYNC_H_
