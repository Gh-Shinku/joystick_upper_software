#ifndef ACTION_H
#define ACTION_H

#define SELF_MADE_HANDLE_R1

#ifdef SELF_MADE_HANDLE_R1
/* 左侧四个按键的左按键和右按键 */
enum class ACTION {
  BUTTON_SHOOT = 4,           /* 发射 */
  BUTTON_AIM_MODE = 5,        /* 瞄准模式  */
  BUTTON_ACC_RPM = 8,         /* 转速微增 */
  BUTTON_DEC_RPM = 9,         /* 转速微减 */
  BUTTON_YAW_CLOCK = 11,      /* 顺时针微调 */
  BUTTON_YAW_ANTI_CLOCK = 10, /* 逆时针微调 */
  BUTTON_DRIBBLE, /* 对于 R1 车来说，DRIBBLE 往后的动作实际上是无效的，全部写上仅为保证编译通过 */
  BUTTON_PAWL,
  BUTTON_SLIDEWAY,
  BUTTON_GO_POINTS,
  BUTTON_FAST_SHOOT,
  BUTTON_KEEP_LOOP,
  ACTION_JOY_SWITCH = 20 /* 手柄摇杆开关 */
};
#else
/* 右移位数 */
enum class ACTION {
  BUTTON_SHOOT = 0,      /* 发射 */
  BUTTON_DRIBBLE = 1,    /* 运球 */
  BUTTON_PAWL = 7,       /* 夹爪开关 */
  BUTTON_SLIDEWAY = 5,   /* 滑轨伸出/收回开关 */
  BUTTON_GO_POINTS = 6,  /* 启动跑点 */
  BUTTON_AIM_MODE = 4,   /* 瞄准模式 */
  BUTTON_FAST_SHOOT = 1, /* 快速发射 */
  BUTTON_KEEP_LOOP = 5   /* 持续转动 */
};
#endif

#endif /* ACTION_H */
