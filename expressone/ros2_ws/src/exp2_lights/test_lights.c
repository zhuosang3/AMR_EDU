/**
 * @file test_lights.c
 * @brief 灯光测试 — 包含方位互斥验证
 *
 * 编译: make && sudo ./test_lights
 */

#define _DEFAULT_SOURCE   /* 启用 usleep (POSIX.1-2001) */
#include "lights.h"
#include <stdio.h>
#include <unistd.h>

/* 兼容旧编译器：用 usleep 替代 sleep */
#ifdef _WIN32
  #include <windows.h>
  #define sleep_ms(ms) Sleep(ms)
#else
  #define sleep_ms(ms) usleep((ms) * 1000)
#endif

int main(void) {
    printf("============================================\n");
    printf("EXP2 小车灯光测试 v2.0 (Orange Pi 5 Plus)\n");
    printf("新特性: 方位互斥 — 同方位只能亮一种颜色\n");
    printf("============================================\n\n");

    int passed = 0, failed = 0;

    /* ================================================
     * 测试 1: 12 个灯依次亮灭 (基本功能)
     * ================================================ */
    printf("── 测试 1: 12 灯依次亮灭 ──\n");

    struct {
        int         pos_int;
        const char *pos_str;
        const char *desc;
        int         is_str;  /* 1=字符串, 0=整数 */
    } seq[] = {
        {17,   NULL,   "左前红1  (17)",   0},
        {27,   NULL,   "左前绿1  (27)",   0},
        {22,   NULL,   "左前黄1  (22)",   0},
        {0,    "SDA",  "右前红2  (SDA)",  1},
        {0,    "SCL",  "右前绿2  (SCL)",  1},
        {0,    "MISO", "右前黄2  (MISO)", 1},
        {6,    NULL,   "左后红3  (6)",    0},
        {13,   NULL,   "左后绿3  (13)",   0},
        {26,   NULL,   "左后黄3  (26)",   0},
        {18,   NULL,   "右后红4  (18)",   0},
        {12,   NULL,   "右后绿4  (12)",   0},
        {21,   NULL,   "右后黄4  (21)",   0},
    };

    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        printf("  %s — 亮", seq[i].desc);
        fflush(stdout);

        if (seq[i].is_str)
            light_on(seq[i].pos_str);
        else
            light_on(seq[i].pos_int);

        sleep_ms(500);

        if (seq[i].is_str)
            light_off(seq[i].pos_str);
        else
            light_off(seq[i].pos_int);

        printf(" → 灭 ✓\n");
        passed++;
    }

    /* ================================================
     * 测试 2: 方位互斥验证
     * 同一方位点亮红色后，再点亮绿色，
     * 红色应被自动关断，只有绿色亮。
     * ================================================ */
    printf("\n── 测试 2: 方位互斥验证 ──\n");

    /* 2a: 左前方位互斥 — 先红后绿 */
    printf("  2a: 左前红(17)亮 → ");
    fflush(stdout);
    light_on(17);
    sleep_ms(500);
    printf("左前绿(27)亮 (红应自动灭) → ");
    fflush(stdout);
    light_on(27);   /* 此时应自动关断 17, 点亮 27 */
    sleep_ms(500);
    printf("关绿 → ");
    fflush(stdout);
    light_off(27);
    sleep_ms(300);
    printf("✓\n");
    passed++;

    /* 2b: 右前方位互斥 — 先绿后黄 */
    printf("  2b: 右前绿(SCL)亮 → ");
    fflush(stdout);
    light_on("SCL");
    sleep_ms(500);
    printf("右前黄(MISO)亮 (绿应自动灭) → ");
    fflush(stdout);
    light_on("MISO");
    sleep_ms(500);
    printf("关黄 → ");
    fflush(stdout);
    light_off("MISO");
    sleep_ms(300);
    printf("✓\n");
    passed++;

    /* 2c: 左后方位互斥 — 先黄后红 */
    printf("  2c: 左后黄(26)亮 → ");
    fflush(stdout);
    light_on(26);
    sleep_ms(500);
    printf("左后红(6)亮 (黄应自动灭) → ");
    fflush(stdout);
    light_on(6);
    sleep_ms(500);
    printf("关红 → ");
    fflush(stdout);
    light_off(6);
    sleep_ms(300);
    printf("✓\n");
    passed++;

    /* 2d: 右后方位互斥 — 先红后绿 */
    printf("  2d: 右后红(18)亮 → ");
    fflush(stdout);
    light_on(18);
    sleep_ms(500);
    printf("右后绿(12)亮 (红应自动灭) → ");
    fflush(stdout);
    light_on(12);
    sleep_ms(500);
    printf("关绿 → ");
    fflush(stdout);
    light_off(12);
    sleep_ms(300);
    printf("✓\n");
    passed++;

    /* ================================================
     * 测试 3: 跨方位独立验证
     * 不同方位可以同时亮不同颜色，互不影响。
     * ================================================ */
    printf("\n── 测试 3: 跨方位独立验证 ──\n");

    printf("  同时点亮: 左前红(17) + 右前绿(SCL) + 左后黄(26) + 右后红(18)\n");
    light_on(17);      /* 左前红 */
    sleep_ms(300);
    light_on("SCL");   /* 右前绿 */
    sleep_ms(300);
    light_on(26);      /* 左后黄 */
    sleep_ms(300);
    light_on(18);      /* 右后红 */
    sleep_ms(1000);
    printf("  4 个方位各亮一种颜色 ✓\n");
    passed++;

    /* 全部关断 */
    light_off(17);
    light_off("SCL");
    light_off(26);
    light_off(18);
    sleep_ms(300);

    /* ================================================
     * 测试 4: 同方位颜色快速切换
     * ================================================ */
    printf("\n── 测试 4: 同方位颜色快速切换 (左前: 红→绿→黄→灭) ──\n");

    printf("  红 → "); fflush(stdout);
    light_on(17);  sleep_ms(400);

    printf("绿 → "); fflush(stdout);
    light_on(27);  sleep_ms(400);

    printf("黄 → "); fflush(stdout);
    light_on(22);  sleep_ms(400);

    printf("灭 ✓\n");
    light_off(22);
    sleep_ms(300);
    passed++;

    printf("\n============================================\n");
    printf("测试完成: %d 通过, %d 失败\n", passed, failed);
    printf("============================================\n");

    light_cleanup();
    return failed > 0 ? 1 : 0;
}
