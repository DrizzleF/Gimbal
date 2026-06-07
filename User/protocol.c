#include "protocol.h"
#include <stdlib.h>

static char buf[64];
static uint8_t idx;

void Protocol_Init(void)
{
    idx = 0;
}

// 兼容两种格式:
// 新: X<err_x>Y<err_y>Z<0|1>\n  — 像素误差 + 有效标志
// 旧: X<abs_x>Y<abs_y>\n        — 绝对像素坐标，隐含有效
uint8_t Protocol_Parse(char ch, CoordFrame *coord)
{
    coord->valid = 0;

    if (ch == '\n')
    {
        buf[idx] = '\0';
        idx = 0;

        char *x_ptr = NULL, *y_ptr = NULL, *z_ptr = NULL;
        if (buf[0] == 'X' || buf[0] == 'x') x_ptr = &buf[1];
        if (x_ptr)
        {
            y_ptr = x_ptr;
            while (*y_ptr && *y_ptr != 'Y' && *y_ptr != 'y') y_ptr++;
            if (*y_ptr) { *y_ptr = '\0'; y_ptr++; }
            else y_ptr = NULL;

            if (y_ptr)
            {
                z_ptr = y_ptr;
                while (*z_ptr && *z_ptr != 'Z' && *z_ptr != 'z') z_ptr++;
                if (*z_ptr) { *z_ptr = '\0'; z_ptr++; }
                else z_ptr = NULL;
            }
        }

        if (x_ptr && y_ptr)
        {
            if (z_ptr)
            {
                // 新格式: 像素误差
                coord->err_x = (int16_t)atoi(x_ptr);
                coord->err_y = (int16_t)atoi(y_ptr);
                coord->valid = (*z_ptr == '1') ? 1 : 0;
            }
            else
            {
                // 旧格式: 绝对坐标 → 算误差，默认有效
                uint16_t abs_x = (uint16_t)atoi(x_ptr);
                uint16_t abs_y = (uint16_t)atoi(y_ptr);
                if (abs_x < 640 && abs_y < 480)
                {
                    coord->err_x = (int16_t)(abs_x - 320);
                    coord->err_y = (int16_t)(abs_y - 240);
                    coord->valid = 1;
                }
            }
            return 1;
        }
    }
    else if (ch != '\r')
    {
        if (idx < sizeof(buf) - 1)
            buf[idx++] = ch;
    }

    return 0;
}
