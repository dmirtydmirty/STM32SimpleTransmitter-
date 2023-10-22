# STM32 Simple transmitter documentation

### Device description

STM32 simple transmitter consists of two parts: a Linux device, that reads file and modulates sequence
and STM32 MCU, that receives samples via UART and converts it into voltage.

In this project the linux device it's OrangePi Zero 2, so a set of cross-compilation tools is required
for example, it could be [this](https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/)

STM32F407 is used as the MCU 

### How to use

This device  can generate signal from file and from sequence than could be declared in code.

At the beginning you needs to set a parameters of a signal:
1) sps - samples per symbol.
2) span - length of shaping impulse in sps ( shaping impulse length = sps*span samples).
3) type of shaping impulse - now available square form, raised cosine and Gaussian impulse.
4) parameter of impulse - it must be more than 0 less than 1, by default it's 0.3, square and RC impulses ignores a parameter.

```c++
uint8_t sps = 5;
uint8_t span = 10;
float param = 0.3;
Transmitter fileTransmitter(sps, span, GAUSSIAN, 0.3);
```
Next you can start generates signal:

from file:

```c++
    fileTransmitter.run(); // from default path file.bin
```
or
```c++
    fileTransmitter.run("/home/user/files/sin.bin"); 
```

from sequence:
```c++
    const uint8_t LEN = 8;
    uint8_t SQ[LEN] = {128, 197, 201, 184, 150, 236, 200, 176};
    uint8_t sps = 5;
    uint8_t span = 10;
    float param = 0.3;
    Transmitter sequenceTransmitter(sps, span, GAUSSIAN, 0.3);
    fileTransmitter.run(SQ, LEN); 
```

### Dependencies 

##### FFTW:

You can download it from [this](https://fftw.org/download.html)

### Elvis:
![pig](https://github.com/dmirtydmirty/STM32SimpleTransmitter-/assets/107511435/378e37ee-9cd4-49c0-8fe3-17cb09a34b5c)

