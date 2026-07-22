/**
 * @file lights.h
 * @brief EXP2 小车灯光控制 — Orange Pi 5 Plus GPIO (纯 C)
 *
 * 每个函数只接受一个参数：GPIO 位置
 *
 * 灯光布局:
 *         左前(1)                    右前(2)
 *     红=17  绿=27  黄=22      红=SDA  绿=SCL  黄=MISO
 *         左后(3)                    右后(4)
 *     红=6   绿=13  黄=26      红=18   绿=12   黄=21
 *
 * === 互斥规则 (v2.0) ===
 * 每个方位(左前/右前/左后/右后)同时只能点亮一种颜色。
 * 调用 light_on() 时会自动关闭同方位的其他颜色，保证硬件级互斥。
 * 例如: 左前红(17)亮 → light_on(27) → 左前红自动灭, 左前绿亮
 *
 * 用法:
 *   #include "lights.h"
 *   light_on(17);        // 左前红 (自动关左前绿/黄)
 *   light_on("SDA");     // 右前红 (自动关右前绿/黄)
 *   light_off(17);       // 左前红灭
 *   light_off("SCL");    // 右前绿灭
 *   light_cleanup();
 */

#ifndef EXP2_LIGHTS_H_
#define EXP2_LIGHTS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ── 底层函数 ── */
void light_init(void);
void light_cleanup(void);
void light_on_int(int position);
void light_on_str(const char * position);
void light_off_int(int position);
void light_off_str(const char * position);

/* ── C11 _Generic: light_on(17) / light_on("IDSD") 统一调用 ── */
#if __STDC_VERSION__ >= 201112L
  #define light_on(pos)  _Generic((pos),         \
      int:          light_on_int,                \
      char *:       light_on_str,                \
      const char *: light_on_str                 \
  )(pos)

  #define light_off(pos) _Generic((pos),         \
      int:          light_off_int,               \
      char *:       light_off_str,               \
      const char *: light_off_str                \
  )(pos)
#endif

#ifdef __cplusplus
}
#endif

#endif  /* EXP2_LIGHTS_H_ */
