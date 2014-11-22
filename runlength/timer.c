#include <time.h>
#include <stdio.h>

static clock_t start, end;

int main() {
    int blah;
    start = clock();
    for (int i = 1; i < 1000000000; i ++){
        blah *= 4;
    }
    end = clock();
    printf("%d\n", end - start);
}
