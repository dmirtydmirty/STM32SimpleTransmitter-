//
// Created by dmirty on 8/11/23.
//

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <system_error>
#include <sys/ioctl.h>
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

#include "../Include/UART.h"


UART::UART() {
    echo_ = false;
    timeout_ms_ = defaultTimeout_ms_;
    baudRateStandard_ = defaultBaudRate_;
    readBufferSize_B_ = defaultReadBufferSize_B_;
    readBuffer_.reserve(readBufferSize_B_);
    state_ = State::CLOSED;
}


UART::UART(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits,
                       FlowControl hardwareFlowControl, FlowControl softwareFlowControl):
        UART() {
    device_ = device;

    baudRateStandard_ = baudRate;
    numDataBits_ = numDataBits;
    parity_ = parity;
    numStopBits_ = numStopBits;
    hardwareFlowControl_ = hardwareFlowControl;
    softwareFlowControl_ = softwareFlowControl;
}

UART::~UART() {
    try {
        Close();
    } catch(std::exception &err) {
        std::cout << err.what() << std::endl;
    }
}

void UART::Open()
{
    if(device_.empty()) {
        throw std::runtime_error("UART error: Attempted to open file when file path has not been assigned to.");
    }
    fileDesc_ = open(device_.c_str(), O_RDWR);

    if(fileDesc_ == -1) {
        throw std::runtime_error("UART error: Could not open device \"" + device_ + "\". Is the device name correct and do you have read/write permissions?");
    }

    ConfigureTermios();

    state_ = State::OPEN;
}

void UART::ConfigureTermios()
{
    termios2 tty = GetTermios2();

    tty.c_cflag     &=  ~CSIZE;
    switch(numDataBits_) {
        case NumDataBits::FIVE:
            tty.c_cflag     |=  CS5;
            break;
        case NumDataBits::SIX:
            tty.c_cflag     |=  CS6;
            break;
        case NumDataBits::SEVEN:
            tty.c_cflag     |=  CS7;
            break;
        case NumDataBits::EIGHT:
            tty.c_cflag     |=  CS8;
            break;
    }

    switch(parity_) {
        case Parity::NONE:
            tty.c_cflag     &=  ~PARENB;
            break;
        case Parity::EVEN:
            tty.c_cflag 	|=   PARENB;
            tty.c_cflag		&=	 ~PARODD;
            break;
        case Parity::ODD:
            tty.c_cflag     |=   PARENB;
            tty.c_cflag		|=	 PARODD;
            break;
    }

    switch(numStopBits_) {
        case NumStopBits::ONE:
            tty.c_cflag     &=  ~CSTOPB;
            break;
        case NumStopBits::TWO:
            tty.c_cflag     |=  CSTOPB;
            break;
    }

    switch(hardwareFlowControl_){
        case FlowControl::OFF:
            tty.c_cflag &= ~CRTSCTS;
            break;

        case FlowControl::ON:
            tty.c_cflag |= CRTSCTS;
            break;
    }

    tty.c_cflag     |=  CREAD | CLOCAL;

        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        switch(baudRateStandard_) {
            case BaudRate::B_0:
                tty.c_ispeed = 0;
                tty.c_ospeed = 0;
                break;
            case BaudRate::B_50:
                tty.c_ispeed = 50;
                tty.c_ospeed = 50;
                break;
            case BaudRate::B_75:
                tty.c_ispeed = 75;
                tty.c_ospeed = 75;
                break;
            case BaudRate::B_110:
                tty.c_ispeed = 110;
                tty.c_ospeed = 110;
                break;
            case BaudRate::B_134:
                tty.c_ispeed = 134;
                tty.c_ospeed = 134;
                break;
            case BaudRate::B_150:
                tty.c_ispeed = 150;
                tty.c_ospeed = 150;
                break;
            case BaudRate::B_200:
                tty.c_ispeed = 200;
                tty.c_ospeed = 200;
                break;
            case BaudRate::B_300:
                tty.c_ispeed = 300;
                tty.c_ospeed = 300;
                break;
            case BaudRate::B_600:
                tty.c_ispeed = 600;
                tty.c_ospeed = 600;
                break;
            case BaudRate::B_1200:
                tty.c_ispeed = 1200;
                tty.c_ospeed = 1200;
                break;
            case BaudRate::B_1800:
                tty.c_ispeed = 1800;
                tty.c_ospeed = 1800;
                break;
            case BaudRate::B_2400:
                tty.c_ispeed = 2400;
                tty.c_ospeed = 2400;
                break;
            case BaudRate::B_4800:
                tty.c_ispeed = 4800;
                tty.c_ospeed = 4800;
                break;
            case BaudRate::B_9600:
                tty.c_ispeed = 9600;
                tty.c_ospeed = 9600;
                break;
            case BaudRate::B_19200:
                tty.c_ispeed = 19200;
                tty.c_ospeed = 19200;
                break;
            case BaudRate::B_38400:
                tty.c_ispeed = 38400;
                tty.c_ospeed = 38400;
                break;
            case BaudRate::B_57600:
                tty.c_ispeed = 57600;
                tty.c_ospeed = 57600;
                break;
            case BaudRate::B_115200:
                tty.c_ispeed = 115200;
                tty.c_ospeed = 115200;
                break;
            case BaudRate::B_230400:
                tty.c_ispeed = 230400;
                tty.c_ospeed = 230400;
                break;
            case BaudRate::B_460800:
                tty.c_ispeed = 460800;
                tty.c_ospeed = 460800;
                break;
        }

    tty.c_oflag     =   0;
    tty.c_oflag     &=  ~OPOST;


    if(timeout_ms_ == -1) {
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
    } else if(timeout_ms_ == 0) {
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
    } else if(timeout_ms_ > 0) {
        tty.c_cc[VTIME] = (cc_t)(timeout_ms_/100);
        tty.c_cc[VMIN] = 0;
    }

    switch(softwareFlowControl_){
        case FlowControl::OFF:
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;

        case FlowControl::ON:
            tty.c_iflag |= (IXON | IXOFF | IXANY);
            break;
    }

    tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_lflag		&= ~ICANON;
    if(echo_) {
        tty.c_lflag |= ECHO;
    } else {
        tty.c_lflag &= ~(ECHO);
    }
    tty.c_lflag		&= ~ECHOE;
    tty.c_lflag		&= ~ECHONL;
    tty.c_lflag		&= ~ISIG;

    this->SetTermios2(tty);
}

void UART::Write(const std::string& data) {
    PortIsOpened(__PRETTY_FUNCTION__);

    int writeResult = write(fileDesc_, data.c_str(), data.size());

    if (writeResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }
}

void UART::WriteBinary(uint8_t const * data, size_t size) {
    PortIsOpened(__PRETTY_FUNCTION__);

    int writeResult = write(fileDesc_, data, size);

    if (writeResult == -1) {
        throw std::system_error(EFAULT, std::system_category());
    }
}

void UART::Read(std::string& data) {
    PortIsOpened(__PRETTY_FUNCTION__);

    ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

    if(n < 0) {
        throw std::system_error(EFAULT, std::system_category());
    }
    else if(n == 0) {
        termios2 term2;
        int rv = ioctl(fileDesc_, TCGETS2, &term2);

        if(rv != 0) {
            throw std::system_error(EFAULT, std::system_category());
        }
    }
    else if(n > 0) {
        data += std::string(&readBuffer_[0], n);
    }
}

void UART::ReadBinary(std::vector<uint8_t>& data) {
    PortIsOpened(__PRETTY_FUNCTION__);

    ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

    if(n < 0) {
        throw std::system_error(EFAULT, std::system_category());
    } else if(n == 0) {
        termios2 term2;
        int rv = ioctl(fileDesc_, TCGETS2, &term2);
        if(rv != 0) {
            throw std::system_error(EFAULT, std::system_category());
        }
    } else if(n > 0) {
        std::copy(readBuffer_.begin(), readBuffer_.begin() + n, back_inserter(data));
    }
}

termios2 UART::GetTermios2()
{
    termios2 term2;

    ioctl(fileDesc_, TCGETS2, &term2);

    return term2;
}

void UART::SetTermios2(termios2 tty)
{
    ioctl(fileDesc_, TCSETS2, &tty);
}

void UART::PortIsOpened(const std::string& prettyFunc) {

    if(state_ != State::OPEN) {
        throw std::runtime_error("UART error: " + std::string() + prettyFunc + " called but state != OPEN. Please call Open() first.");
    }

    if(fileDesc_ < 0) {
        throw std::runtime_error("UART error: " +std::string() + prettyFunc + " called but file descriptor < 0, indicating file has not been opened.");
    }
}

void UART::Close() {
    if(fileDesc_ != -1) {
        auto retVal = close(fileDesc_);
        if(retVal != 0)
            throw std::runtime_error("UART error: Tried to close serial port " + device_ + ", but close() failed.");

        fileDesc_ = -1;
    }
    state_ = State::CLOSED;
}

int32_t UART::Available() {
    PortIsOpened(__PRETTY_FUNCTION__);

    int32_t ret = 0;
    ioctl(fileDesc_, FIONREAD, &ret);
    return ret;

}
State UART::GetState() {
    return state_;
}
