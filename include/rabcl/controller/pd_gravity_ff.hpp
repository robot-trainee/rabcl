#ifndef RABCL__CONTROLLER__PD_GRAVITY_FF_HPP_
#define RABCL__CONTROLLER__PD_GRAVITY_FF_HPP_

namespace rabcl
{
class PdGravityFf
{
public:
  PdGravityFf(float kp, float kd, float gravity_ff, float output_max);
  ~PdGravityFf();
  float Calc(float pos_target, float pos_actual, float vel_actual);
  void SetGains(float kp, float kd, float gravity_ff);

private:
  float kp_;
  float kd_;
  float gravity_ff_;
  float output_max_;
};
}  // namespace rabcl

#endif  // RABCL__CONTROLLER__PD_GRAVITY_FF_HPP_
