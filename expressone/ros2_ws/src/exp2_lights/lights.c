/**
 * @file lights.c
 * @brief 灯光 GPIO 控制 — libgpiod C API (v2.0 带方位互斥)
 *
 * 编译: gcc -std=c11 -c lights.c -o lights.o
 * 链接: gcc test_lights.c lights.o -lgpiod -o test_lights
 *
 * === 互斥规则 ===
 * 每个方位(左前/右前/左后/右后)同时只能点亮一种颜色。
 * light_on() 会自动关闭同方位其他颜色，保证硬件级互斥。
 */

#include "lights.h"

#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * 引脚映射
 * 用户标识 → {gpiochip编号, line偏移量, 描述}
 * ============================================================ */

typedef struct {
    int         chip;   /* /dev/gpiochipN */
    int         line;   /* line offset   */
    const char *desc;   /* 中文描述       */
} PinInfo;

/* 利用内核 GPIO 编号唯一标识一个 pin: id = (chip << 16) | line */
#define PIN_ID(chip, line)  (((chip) << 16) | (line))
#define PIN_CHIP(id)        ((id) >> 16)
#define PIN_LINE(id)        ((id) & 0xFFFF)

/* ── 数字引脚(9个, RPi BCM GPIO 编号) ── */
static const struct {
    int      pos;      /* 用户传入的整数 */
    PinInfo  info;
} kIntPins[] = {
    {17, {1,  4, "左前红1  GPIO1_A4  物理11"}},
    {27, {1,  7, "左前绿1  GPIO1_A7  物理13"}},
    {22, {1,  8, "左前黄1  GPIO1_B0  物理15"}},
    {6,  {3,  0, "左后红3  GPIO3_A0  物理31"}},
    {13, {3, 18, "左后绿3  GPIO3_C2  物理33"}},
    {26, {3, 17, "左后黄3  GPIO3_C1  物理37"}},
    {18, {3,  1, "右后红4  GPIO3_A1  物理12"}},
    {12, {1,  3, "右后绿4  GPIO1_A3  物理32"}},
    {21, {3,  3, "右后黄4  GPIO3_A3  物理40"}},
};

/* ── 命名引脚(3个, 排针功能名) ── */
static const struct {
    const char *pos;      /* 用户传入的字符串 */
    PinInfo     info;
} kStrPins[] = {
    {"SDA",  {0, 16, "右前红2  SDA.2       物理3 "}},
    {"SCL",  {0, 15, "右前绿2  SCL.2       物理5 "}},
    {"MISO", {1,  9, "右前黄2  SPI0_RXD    物理21"}},
};

#define K_NUM_INT  (sizeof(kIntPins) / sizeof(kIntPins[0]))
#define K_NUM_STR  (sizeof(kStrPins) / sizeof(kStrPins[0]))
#define K_TOTAL    (K_NUM_INT + K_NUM_STR)

/* ============================================================
 * 方位定义 — 每个方位有 3 种颜色引脚，同时只能亮一种
 * ============================================================ */
typedef enum {
    POS_NONE = 0,
    POS_LEFT_FRONT  = 1,  /* 左前: 17(R), 27(G), 22(Y) */
    POS_RIGHT_FRONT = 2,  /* 右前: SDA(R), SCL(G), MISO(Y) */
    POS_LEFT_BACK   = 3,  /* 左后: 6(R), 13(G), 26(Y) */
    POS_RIGHT_BACK  = 4,  /* 右后: 18(R), 12(G), 21(Y) */
} LightPosition;

static const char * position_name(LightPosition p) {
    switch (p) {
        case POS_LEFT_FRONT:  return "左前";
        case POS_RIGHT_FRONT: return "右前";
        case POS_LEFT_BACK:   return "左后";
        case POS_RIGHT_BACK:  return "右后";
        default:              return "未知";
    }
}

/* 根据 chip+line 判断所属方位 */
static LightPosition get_position(int chip, int line) {
    int id = PIN_ID(chip, line);

    /* 左前: 17(1,4), 27(1,7), 22(1,8) */
    if (id == PIN_ID(1,4) || id == PIN_ID(1,7) || id == PIN_ID(1,8))
        return POS_LEFT_FRONT;

    /* 右前: SDA(0,16), SCL(0,15), MISO(1,9) */
    if (id == PIN_ID(0,16) || id == PIN_ID(0,15) || id == PIN_ID(1,9))
        return POS_RIGHT_FRONT;

    /* 左后: 6(3,0), 13(3,18), 26(3,17) */
    if (id == PIN_ID(3,0) || id == PIN_ID(3,18) || id == PIN_ID(3,17))
        return POS_LEFT_BACK;

    /* 右后: 18(3,1), 12(1,3), 21(3,3) */
    if (id == PIN_ID(3,1) || id == PIN_ID(1,3) || id == PIN_ID(3,3))
        return POS_RIGHT_BACK;

    return POS_NONE;
}

/* ============================================================
 * GPIO 资源管理
 * ============================================================ */

static struct {
    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int         id;    /* PIN_ID */
} g_lines[K_TOTAL];

static int g_count     = 0;
static int g_inited    = 0;

/* 按 int position 查找 PinInfo */
static const PinInfo * find_by_int(int pos) {
    for (size_t i = 0; i < K_NUM_INT; i++) {
        if (kIntPins[i].pos == pos) return &kIntPins[i].info;
    }
    return NULL;
}

/* 按字符串 position 查找 PinInfo */
static const PinInfo * find_by_str(const char * pos) {
    for (size_t i = 0; i < K_NUM_STR; i++) {
        if (strcmp(kStrPins[i].pos, pos) == 0) return &kStrPins[i].info;
    }
    return NULL;
}

/* 初始化单个引脚 */
static int init_one(const PinInfo * pin) {
    int id = PIN_ID(pin->chip, pin->line);

    /* 检查是否已初始化 */
    for (int i = 0; i < g_count; i++) {
        if (g_lines[i].id == id) return 0;
    }

    char path[64];
    snprintf(path, sizeof(path), "/dev/gpiochip%d", pin->chip);

    struct gpiod_chip *chip = gpiod_chip_open(path);
    if (!chip) {
        fprintf(stderr, "[exp2_lights] 错误: 无法打开 %s (需要 root)\n", path);
        return -1;
    }

    struct gpiod_line *line = gpiod_chip_get_line(chip, pin->line);
    if (!line) {
        fprintf(stderr, "[exp2_lights] 错误: 无法获取 %s line %d\n",
                path, pin->line);
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_request_output(line, "exp2_lights", 0) < 0) {
        fprintf(stderr, "[exp2_lights] 错误: %s:%d 设置输出失败 (可能被占用)\n",
                path, pin->line);
        gpiod_chip_close(chip);
        return -1;
    }

    g_lines[g_count].chip = chip;
    g_lines[g_count].line = line;
    g_lines[g_count].id   = id;
    g_count++;

    fprintf(stderr, "[exp2_lights] %s (gpiochip%d:%d)\n",
            pin->desc, pin->chip, pin->line);
    return 0;
}

/* ============================================================
 * 公开接口
 * ============================================================ */

void light_init(void) {
    if (g_inited) return;

    for (size_t i = 0; i < K_NUM_INT; i++)
        init_one(&kIntPins[i].info);
    for (size_t i = 0; i < K_NUM_STR; i++)
        init_one(&kStrPins[i].info);

    g_inited = 1;
    fprintf(stderr, "[exp2_lights] 初始化完成: %d/%zu 个 GPIO\n",
            g_count, K_TOTAL);
}

void light_cleanup(void) {
    for (int i = 0; i < g_count; i++) {
        gpiod_line_set_value(g_lines[i].line, 0);
        gpiod_line_release(g_lines[i].line);
        gpiod_chip_close(g_lines[i].chip);
    }
    g_count  = 0;
    g_inited = 0;
    fprintf(stderr, "[exp2_lights] GPIO 资源已释放\n");
}

/* 确保已初始化 */
static inline void ensure_init(void) {
    if (!g_inited) light_init();
}

/* ── 按 chip+line 写值 (不触发互斥, 内部使用) ── */
static void write_line_raw(int chip, int line, int value) {
    int id = PIN_ID(chip, line);
    ensure_init();
    for (int i = 0; i < g_count; i++) {
        if (g_lines[i].id == id) {
            gpiod_line_set_value(g_lines[i].line, value);
            return;
        }
    }
    fprintf(stderr, "[exp2_lights] 错误: 引脚 gpiochip%d:%d 未初始化\n",
            chip, line);
}

/* ── 互斥关断: 关闭与 pin 同方位的其他颜色引脚 ── */
static void mutex_off_siblings(const PinInfo *pin) {
    LightPosition target_pos = get_position(pin->chip, pin->line);
    if (target_pos == POS_NONE) return;

    int pin_id = PIN_ID(pin->chip, pin->line);

    /* 遍历所有 int 引脚，关闭同方位不同引脚 */
    for (size_t i = 0; i < K_NUM_INT; i++) {
        int other_id = PIN_ID(kIntPins[i].info.chip, kIntPins[i].info.line);
        if (other_id == pin_id) continue;  /* 跳过自己 */
        if (get_position(kIntPins[i].info.chip, kIntPins[i].info.line) == target_pos) {
            write_line_raw(kIntPins[i].info.chip, kIntPins[i].info.line, 0);
        }
    }

    /* 遍历所有 str 引脚，关闭同方位不同引脚 */
    for (size_t i = 0; i < K_NUM_STR; i++) {
        int other_id = PIN_ID(kStrPins[i].info.chip, kStrPins[i].info.line);
        if (other_id == pin_id) continue;  /* 跳过自己 */
        if (get_position(kStrPins[i].info.chip, kStrPins[i].info.line) == target_pos) {
            write_line_raw(kStrPins[i].info.chip, kStrPins[i].info.line, 0);
        }
    }

    fprintf(stderr, "[exp2_lights] 互斥: %s 方位已关断其他颜色\n",
            position_name(target_pos));
}

void light_on_int(int position) {
    const PinInfo *p = find_by_int(position);
    if (!p) {
        fprintf(stderr, "[exp2_lights] 无效位置: %d (有效: 17,27,22,6,13,26,18,12,21)\n",
                position);
        return;
    }
    /* 先关同方位其他颜色，再点亮目标 */
    mutex_off_siblings(p);
    write_line_raw(p->chip, p->line, 1);
}

void light_off_int(int position) {
    const PinInfo *p = find_by_int(position);
    if (!p) {
        fprintf(stderr, "[exp2_lights] 无效位置: %d (有效: 17,27,22,6,13,26,18,12,21)\n",
                position);
        return;
    }
    write_line_raw(p->chip, p->line, 0);
}

void light_on_str(const char * position) {
    const PinInfo *p = find_by_str(position);
    if (!p) {
        fprintf(stderr, "[exp2_lights] 无效位置: \"%s\" (有效: SDA,SCL,MISO)\n",
                position);
        return;
    }
    /* 先关同方位其他颜色，再点亮目标 */
    mutex_off_siblings(p);
    write_line_raw(p->chip, p->line, 1);
}

void light_off_str(const char * position) {
    const PinInfo *p = find_by_str(position);
    if (!p) {
        fprintf(stderr, "[exp2_lights] 无效位置: \"%s\" (有效: SDA,SCL,MISO)\n",
                position);
        return;
    }
    write_line_raw(p->chip, p->line, 0);
}
