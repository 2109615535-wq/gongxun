/* Host-side protocol interoperability runner; no HAL, no motors. */
#include "rescue_vision.h"
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

int main(int argc, char **argv) {
    RV_Receiver receiver;
    RV_Config config;
    unsigned char header[6], byte;
    uint32_t now;
    unsigned length, i;
    RV_Command command;
#ifdef _WIN32
    /* Protocol bytes may contain Ctrl-Z/CRLF; Windows stdin must be binary. */
    _setmode(0, _O_BINARY);
#endif
    config.bumper_forward_mm = argc > 1 ? (float)atof(argv[1]) : 120;
    config.collection_center_left_mm = argc > 2 ? (float)atof(argv[2]) : 0;
    config.allow_forward = argc > 3 ? (uint8_t)atoi(argv[3]) : 1;
    RV_Init(&receiver, config);
    while (fread(header, 1, 6, stdin) == 6) {
        now = (uint32_t)header[0] | ((uint32_t)header[1] << 8) |
              ((uint32_t)header[2] << 16) | ((uint32_t)header[3] << 24);
        length = (unsigned)header[4] | ((unsigned)header[5] << 8);
        for (i = 0; i < length; ++i) {
            if (fread(&byte, 1, 1, stdin) != 1) return 2;
            RV_OnByte(&receiver, byte, now);
        }
        command = RV_Step(&receiver, now);
        printf("%d %d %.5f %.5f %.5f %.5f %lu %lu\n", (int)command.state,
               (int)command.reason, command.forward_mm_s, command.clockwise_rad_s,
               command.gap_mm, command.lateral_mm,
               (unsigned long)receiver.duplicates, (unsigned long)receiver.crc_errors);
    }
    return 0;
}
