/*
Module for decoding a runlength-encoded data stream from 
the pru
*/

#include "MT9M001_i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "utilities.h"

/*
encoded: pointer to beginning of encoded frame
decoded: pointer to array to hold decoded frame, assumed to be initialized
to all 0
*/
void decodeFrame(uint8_t *encoded, uint8_t *decoded, int height, int width) {
    int i = 0; // byte index into decoded
    int j = 0; // byte index into encoded
    while (i < height * width) {
        if (encoded[j] == 255) {
            // increment i by the number of pixels in the run
            // beware of little-endianness
            i += 4 * (encoded[j + 1] + 0x100 * encoded[j + 2] + 0x10000 * encoded[j + 3]);
        } else {
            ((uint32_t *) decoded)[i/4] = ((uint32_t *) encoded)[j/4];
            i += 4;
        }
        j += 4;
    }
    if (((uint32_t *) encoded)[j/4 + 1] != 0) {
        printf("frame end sequence absent in expected location, aborting. \n");
        exit(1);
    }
}

int main(int argc, char **argv) {
    char *fname = NULL;
    char *outname = NULL;
    uint8_t *decoded;
    uint8_t *encoded;
    FILE *f = NULL;

    if (argc != 2) {
        printf("usage: ./%s filename", argv[0]);
        exit(1);
    } 
    fname = argv[1];

    if ((f = fopen(fname, "rb")) == NULL) {
        printf("Can't open file: %s \n", fname);
        exit(1);
    }

    decoded = malloc(sizeof(uint8_t) * MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH);
    fread(encoded, sizeof(encoded[0]), MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH / sizeof(encoded[0]), \
            f);
    decodeFrame(encoded, decoded, MT9M001_MAX_HEIGHT, MT9M001_MAX_WIDTH);
    fclose(f);

    outname =  concatStr("decoded_", fname, 128);
    exposureWrite32(outname, (uint32_t *) decoded, MT9M001_MAX_HEIGHT * MT9M001_MAX_WIDTH * (sizeof(uint32_t) / sizeof(decoded[0])));
}
