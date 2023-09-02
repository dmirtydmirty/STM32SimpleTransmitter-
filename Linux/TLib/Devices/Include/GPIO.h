//
// Created by dmirty on 8/11/23.
//

#ifndef LINUX_GPIO_H
#define LINUX_GPIO_H

#include <string>
#include <fstream>

#define GPIO_IN "in"
#define GPIO_OUT "out"

enum GPIOPinLevel{
    GPIOHigth = 1,
    GPIOLow = 0
};


class GPIO
{
    inline static const std::string GPIOroot = "/sys/class/gpio/";
    const int id;
public:
    GPIO(int id_, const char* dir): id(id_)
    {
        std::string exportFile =GPIOroot + "export";
        std::string directionFile = GPIOroot + + "gpio" + std::to_string(id) + "/direction";
        std::fstream GPIOInitPin(exportFile, std::ios::out);
        if (!GPIOInitPin.is_open()) throw std::runtime_error("Couldn't open file: " + exportFile);
        GPIOInitPin << std::to_string(id);
        GPIOInitPin.close();

        std::fstream GPIOInitDir(directionFile, std::ios::out);
        if (!GPIOInitDir.is_open()) throw std::runtime_error("Couldn't open file: " + directionFile);
        GPIOInitDir << dir;
        GPIOInitDir.close();
    }
    void GPIOPinSet(const GPIOPinLevel state) const
    {
        std::string valueFile = GPIOroot + "gpio" + std::to_string(id) + "/value";
        std::fstream GPIOValueInit(valueFile, std::ios::out);
        if (!GPIOValueInit.is_open()) throw std::runtime_error("Couldn't open file: " + valueFile);
        GPIOValueInit << std::to_string(state);
        GPIOValueInit.close();
    }
    GPIOPinLevel GPIOPinRead() const
    {
        GPIOPinLevel state;
        std::string valueFileContent;
        std::string valueFile = GPIOroot + "gpio" + std::to_string(id) + "/value";
        std::fstream GPIOValueRead(valueFile, std::ios::in);
        if (!GPIOValueRead.is_open()) throw std::runtime_error("Couldn't open file: " + valueFile);
        GPIOValueRead >> valueFileContent;
        if (valueFileContent == "1") return GPIOHigth;
        else if (valueFileContent == "0") return GPIOLow;
        else  throw std::runtime_error("Incorrect file " + valueFile);
        GPIOValueRead.close();
        return state;
    }
    void GPIOPinToggle()
    {
        GPIOPinLevel state = GPIOPinRead();
        if (state == GPIOLow) GPIOPinSet(GPIOHigth);
        if (state == GPIOHigth) GPIOPinSet(GPIOLow);
    }
    ~GPIO(){
        this->GPIOPinSet(GPIOLow);
        std::string unexportFile =GPIOroot + "unexport";
        std::fstream GPIODeinitPin(unexportFile, std::ios::out);
        GPIODeinitPin << id;
        GPIODeinitPin.close();
    }
};

#endif //LINUX_GPIO_H
