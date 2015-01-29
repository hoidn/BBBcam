#ifndef UTILITIES_H
#define UTILITIES_H
 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void exposureWrite32(char *fname, uint32_t *arr, int arrSize);
char *concatStr(char *str1, char *str2, int bufSize);

#endif
