/*
Module for decoding a runlength-encoded data stream from 
the pru
*/

#include <stdio.h>
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
            i += 4 * (encoded[j + 1] + 0x100 * encoded[r + 2] + 0x10000 * encoded[j + 3]);
        } else {
            ((uint32_t *) decoded)[i/4] = ((uint32_t *) encoded)[j/4];
            i += 4;
        }
        j += 4;
    }
    if (((uint32_t *) encoded)[j/4 + 1] != 0) {
        printf("frame end sequence absent in expected location, aborting. \n");
    }
}
    
