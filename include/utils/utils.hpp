#ifndef HEADER_UTILS_H
#define HEADER_UTILS_H

#include <cmath>
#include <type_traits>

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type clamp(const T &val, const T &min, const T &max) {
  if (val < min)
    return min;
  else if (val > max)
    return max;
  else
    return val;
}

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type normalizeRad(const T &rad) {
  T normalized_rad = rad;
  while (normalized_rad > M_PI) normalized_rad -= 2 * M_PI;
  while (normalized_rad < -M_PI) normalized_rad += 2 * M_PI;
  return normalized_rad;
}

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type radToDeg(const T &rad) {
  return rad * 180.0 / M_PI;
}

template <typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type degToRad(const T &deg) {
  return deg * M_PI / 180.0;
}

/* 位置式 PID 控制器实现  Position-based PID */
class PID_Controller {
private:
  double kp_;
  double ki_;
  double kd_;
  double err_last_;
  double err_total_;
  double limit_output_;
  double output_;

public:
  PID_Controller(const double &kp, const double &ki, const double &kd, const double &limit_output)
      : kp_(kp), ki_(ki), kd_(kd), limit_output_(limit_output) {}

  double update(const double err_cur) {
    err_total_ += err_cur;
    output_ = kp_ * err_cur + ki_ * err_total_ + kd_ * (err_cur - err_last_);
    output_ = clamp(output_, -limit_output_, limit_output_);
    err_last_ = err_cur;
    return output_;
  }

  void set_pid(const double &kp, const double &ki, const double &kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void reset() {
    err_last_ = 0;
    err_total_ = 0;
  }
};

#endif /* HEADER_UTILS_H */