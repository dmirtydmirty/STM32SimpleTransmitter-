//
// Created by dmirty on 9/2/23.
//

#ifndef TRANSMITTER_TRANSMITTER_H
#define TRANSMITTER_TRANSMITTER_H

#include <string>
#include <thread>

#include "Modulator.h"
#include "UART.h"
#include "GPIO.h"


class Transmitter{
    std::string uart_ = "/dev/ttyS5"; // UART5
    std::string file;
    static  inline const std::string DEFAULT_FILE = "file.bin";
    struct ModulatorParams{
        uint8_t sps_;
        uint8_t span_;
        shaping_impulse_type impulse_form_;
        float param_;
    } modulatorParams;
public:
    Transmitter(std::string uart, uint8_t sps, uint8_t span, shaping_impulse_type impulse_form, float param =  0.3  );
    Transmitter(uint8_t sps, uint8_t span, shaping_impulse_type impulse_form, float param =  0.3  );
    void run(std::string file = DEFAULT_FILE);
    void run(uint8_t* head, size_t size);
};

#endif //TRANSMITTER_TRANSMITTER_H
