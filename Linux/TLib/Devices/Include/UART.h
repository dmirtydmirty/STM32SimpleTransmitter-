//
// Created by dmirty on 8/11/23.
//

#ifndef LINUX_UART_H
#define LINUX_UART_H

// System headers
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>

enum BaudRate {
    B_0,
    B_50,
    B_75,
    B_110,
    B_134,
    B_150,
    B_200,
    B_300,
    B_600,
    B_1200,
    B_1800,
    B_2400,
    B_4800,
    B_9600,
    B_19200,
    B_38400,
    B_57600,
    B_115200,
    B_230400,
    B_460800,
};


enum NumDataBits {
    FIVE,
    SIX,
    SEVEN,
    EIGHT,
};

enum Parity {
    NONE,
    EVEN,
    ODD,
};

enum NumStopBits {
    ONE,
    TWO,
};

enum FlowControl {
    OFF,
    ON,
};


enum class State {
    CLOSED,
    OPEN,
};

class UART {

public:

    UART();

    UART(const std::string &device, BaudRate baudRate = defaultBaudRate_, NumDataBits numDataBits = defaultNumDataBits,
               Parity parity = defaultParity, NumStopBits numStopBits = defaultNumStopBits,
               FlowControl hardwareFlowControl = defaultHardwareFlowControl, FlowControl softwareFlowControl = defaultSoftwareFlowControl);

    virtual ~UART();

    void Open();

    void Close();

    void Write(const std::string& data);

    void WriteBinary(uint8_t const * data, size_t size = 1 );

    void Read(std::string& data);

    void ReadBinary(std::vector<uint8_t>& data);

    State GetState();

    int32_t Available();

private:

    void ConfigureTermios();

    termios2 GetTermios2();

    void SetTermios2(termios2 tty);

    void PortIsOpened(const std::string& prettyFunc);

    State state_;

    std::string device_;

    BaudRate baudRateStandard_;

    NumDataBits numDataBits_;

    Parity parity_ = Parity::NONE;

    NumStopBits numStopBits_;

    FlowControl hardwareFlowControl_;

    FlowControl softwareFlowControl_;

    int fileDesc_;

    bool echo_;

    int32_t timeout_ms_;

    std::vector<char> readBuffer_;
    unsigned char readBufferSize_B_;

    static constexpr BaudRate defaultBaudRate_ = BaudRate::B_57600;
    static constexpr int32_t defaultTimeout_ms_ = -1;
    static constexpr unsigned char defaultReadBufferSize_B_ = 255;
    static constexpr  NumDataBits defaultNumDataBits = NumDataBits::EIGHT;
    static constexpr Parity defaultParity = Parity::NONE;
    static constexpr NumStopBits defaultNumStopBits = NumStopBits::ONE;
    static constexpr FlowControl defaultHardwareFlowControl = FlowControl::OFF;
    static constexpr FlowControl defaultSoftwareFlowControl = FlowControl::OFF;
};

#endif //LINUX_UART_H
