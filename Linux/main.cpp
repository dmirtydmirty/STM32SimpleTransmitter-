#include <cstdint>
#include "Include/Transmitter.h"





int main() {

    const uint8_t LEN = 8;
    uint8_t SQ[LEN] = {128, 197, 201, 184, 150, 236, 200, 176};
    uint8_t sps = 5;
    uint8_t span = 10;
    float param = 0.3;
    Transmitter fileTransmitter(sps, span, GAUSSIAN, 0.3);
    fileTransmitter.run(SQ, LEN);

    return 0;
}
