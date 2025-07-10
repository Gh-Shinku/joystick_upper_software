#ifndef ACTION_H
#define ACTION_H

#ifdef SELF_MADE_HANDLE_R1
enum class ACTION {
  BUTTON_SHOOT = 4,           /* 发射 */
  BUTTON_AIM_MODE = 5,        /* 瞄准模式  */
  BUTTON_ACC_RPM = 8,         /* 转速微增 */
  BUTTON_DEC_RPM = 9,         /* 转速微减 */
  BUTTON_YAW_CLOCK = 11,      /* 顺时针微调 */
  BUTTON_YAW_ANTI_CLOCK = 10, /* 逆时针微调 */
  BUTTON_DRIBBLE,        /* 对于 R1 车来说，DRIBBLE 往后的动作实际上是无效的，全部写上仅为保证编译通过 */
  BUTTON_PAWL,           /* 夹爪开关 */
  BUTTON_SLIDEWAY,       /* 滑轨伸出/收回开关 */
  BUTTON_GO_POINTS,      /* 启动跑点 */
  BUTTON_FAST_SHOOT,     /* 快速发射 */
  BUTTON_KEEP_LOOP,      /* 持续转动 */
  ACTION_JOY_SWITCH = 20 /* 手柄摇杆开关 */
};
#else
/* R2 */
enum class ACTION {
  BUTTON_SHOOT,          /* 发射 */
  BUTTON_AIM_MODE,       /* 瞄准模式  */
  BUTTON_ACC_RPM,        /* 转速微增 */
  BUTTON_DEC_RPM,        /* 转速微减 */
  BUTTON_YAW_CLOCK,      /* 顺时针微调 */
  BUTTON_YAW_ANTI_CLOCK, /* 逆时针微调 */
  BUTTON_DRIBBLE = 5,    /* 对于 R1 车来说，DRIBBLE 往后的动作实际上是无效的，全部写上仅为保证编译通过 */
  BUTTON_PAWL = 6,       /* 夹爪开关 */
  BUTTON_SLIDEWAY = 7,   /* 滑轨伸出/收回开关 */
  BUTTON_GO_POINTS = 8,  /* 启动跑点 */
  BUTTON_FAST_SHOOT = 9, /* 快速发射 */
  BUTTON_KEEP_LOOP = 10, /* 持续转动 */
  ACTION_JOY_SWITCH      /* 手柄摇杆开关 */
};
#endif

#endif /* ACTION_H */
