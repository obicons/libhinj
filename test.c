#include <stdio.h>
#include <stdlib.h>
#include "libhinj.h"

int
main(void)
{
        float mag0 = 0, mag1 = 0, mag2 = 0;
        int e;

        printf("sizeof(struct compass_pkt) = %d\n", sizeof(struct compass_pkt));

        if ((e = update_compass(&mag0, &mag1, &mag2, 0)))
                fprintf(stderr, "%s\n", hinj_strerror(e));
}
