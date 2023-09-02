//
// Created by dmirty on 9/2/23.
//

#include "../Include/Transmitter.h"

Transmitter::Transmitter(std::string uart, uint8_t sps, uint8_t span, shaping_impulse_type impulse_form, float param) {
    uart_= uart;
    modulatorParams = {sps, span, impulse_form, param};
};

Transmitter::Transmitter(uint8_t sps, uint8_t span, shaping_impulse_type impulse_form, float param) {
    modulatorParams = {sps, span, impulse_form, param};
}

void Transmitter::run(std::string file) {

    uint8_t oneByte;
    std::vector<uint8_t> fileContent;

    std::fstream fin(file, std::ios::in|std::ios::binary);
    while(!fin.eof()){
        fin.read(reinterpret_cast<char *>(&oneByte), sizeof(uint8_t));
        fileContent.push_back(oneByte);
    }
    modulator BPSKmod(BPSK);
    dsp_result samples = BPSKmod.exec(&fileContent[0], fileContent.size(), modulatorParams.sps_, modulatorParams.span_,
                                      modulatorParams.impulse_form_, modulatorParams.param_);

    UART uart(uart_, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    uart.Open();
    float sample;
    GPIO pin(73, GPIO_OUT);
    pin.GPIOPinSet(GPIOHigth);

    for (int i = 0; i < samples.size; ++i) {
        sample = samples.head[i].real();
        std::cout << sample << " " << i+1 << std::endl;
        uart.WriteBinary(reinterpret_cast<uint8_t *>(&sample), sizeof(sample));
        std::this_thread::sleep_for(std::chrono::milliseconds (50));
    }
    pin.GPIOPinSet(GPIOLow);
}

void Transmitter::run(uint8_t *head, size_t size) {
    modulator BPSKmod(BPSK);

    dsp_result samples = BPSKmod.exec(head, size, modulatorParams.sps_, modulatorParams.span_,
                                      modulatorParams.impulse_form_, modulatorParams.param_);

    UART uart(uart_, BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    uart.Open();
    float sample;
    GPIO pin(73, GPIO_OUT);
    pin.GPIOPinSet(GPIOHigth);

    for (int i = 0; i < samples.size; ++i) {
        sample = samples.head[i].real();
        std::cout << sample << " " << i+1 << std::endl;
        uart.WriteBinary(reinterpret_cast<uint8_t *>(&sample), sizeof(sample));
        std::this_thread::sleep_for(std::chrono::milliseconds (50));
    }
    pin.GPIOPinSet(GPIOLow);
}