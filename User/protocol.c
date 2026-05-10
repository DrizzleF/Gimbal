#include "protocol.h"
#include <stdlib.h>
#include <string.h>

static char buf[32];
static uint8_t idx;

void Protocol_Init(void)
{
    idx = 0;
}

uint8_t Protocol_Parse(char ch, CoordFrame *coord)
{
    coord->valid = 0;

    if (ch == '\n')
    {
        buf[idx] = '\0';
        idx = 0;

        if (buf[0] == 'X')
        {
            char *y_ptr = strchr(buf, 'Y');
            if (y_ptr != NULL)
            {
                *y_ptr = '\0';
                coord->target_x = (uint16_t)atoi(&buf[1]);
                coord->target_y = (uint16_t)atoi(y_ptr + 1);
                coord->valid = 1;
                return 1;
            }
        }
    }
    else if (ch != '\r')
    {
        if (idx < sizeof(buf) - 1)
            buf[idx++] = ch;
    }

    return 0;
}
