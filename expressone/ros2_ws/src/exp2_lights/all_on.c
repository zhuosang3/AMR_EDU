/**
 * @file all_on.c
 * @brief 点亮所有 4 个方位的灯 (每个方位一种颜色，默认红色)
 *
 * 编译: gcc -std=c11 all_on.c lights.o -lgpiod -o all_on
 * 运行: sudo ./all_on
 */

#include "lights.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    printf("EXP2 灯光全开 (v2.0 方位互斥模式)\n");
    printf("左前红 + 右前红 + 左后红 + 右后红\n");

    light_on(17);       /* 左前红 */
    light_on("SDA");    /* 右前红 */
    light_on(6);        /* 左后红 */
    light_on(18);       /* 右后红 */

    printf("4 方位已全部点亮红色，按 Enter 关闭...\n");
    getchar();

    light_off(17);
    light_off("SDA");
    light_off(6);
    light_off(18);

    light_cleanup();
    printf("已关闭并清理。\n");
    return 0;
}
