#include <stdio.h>

#define F_CPU 16000000.0

// AVR 2x USART:    BAUD = F_CPU/8/(UBRR+1)
//                  UBRR = F_CPU/8/BAUD-1

int divs[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 16, 40, -1 };

int main(int argc, char **argv) {
    int i, p, n, ubrr_p, baud_p, ubrr_n, baud_n;
    float pal = 1773447, ntsc = 1789790, avg = 1781618.5;

    printf("POKEY\t\t   PAL\t  UBRR\t  BAUD\t\t  NTSC\t  UBRR\t  BAUD\n");

    for (i=0; divs[i] != -1; i++) {
        p = (int)( pal/(2*(7+divs[i]))+0.5);
        n = (int)(ntsc/(2*(7+divs[i]))+0.5);
        ubrr_p = (int)((F_CPU/8.0/p-1.0)+0.5);
        baud_p = (int)((F_CPU/8/(ubrr_p+1))+0.5);
        ubrr_n = (int)((F_CPU/8.0/n-1.0)+0.5);
        baud_n = (int)((F_CPU/8/(ubrr_n+1))+0.5);
        printf("$%02x (%2i)\t%6i\t%6i\t%6i\t\t%6i\t%6i\t\%6i\n",
                divs[i], divs[i], p, ubrr_p, baud_p, n, ubrr_n, baud_n);
    }
    return 0;
}
