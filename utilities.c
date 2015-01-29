#include "utilities.h"


// return char * to new string that's a concatenation of str1 and str2
char *concatStr(char *str1, char *str2, int bufSize) {
    char *nameBuffer = malloc(sizeof(char) * bufSize);
    strncpy(nameBuffer, str1, bufSize);
    strncat(nameBuffer, str2, bufSize);
    return nameBuffer;
}


//write uint32_t array to file
void exposureWrite32(char *fname, uint32_t *arr, int arrSize) {
     FILE *fp;
     fp = fopen(fname, "wb");
     fwrite(arr, sizeof(uint32_t), arrSize, fp);
     //fclose(fp);
     printf(fname);
     printf("\n");
}
