/*


LICENSE



This controller is based on the work detailed in the following publication:

Z. Wu∗, S. Cheng∗, P. Zhao, A. Gahlawat, K. A. Ackerman, A. Lakshmanan, C. Yang, J. Yu, and N. Hovakimyan:
L1 adaptive augmentation of geometric control for agile quadrotors with performance guarantees,
arXiv preprint arXiv:2302.07208, 2023


The baseline controller is based on the 'mellinger' controller included elsewhere in this firmware, which is itself based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.


The following modifications to this publication were made in 'controller_mellinger.c':
* Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
* D-term for angular velocity
* Support to use this controller as an attitude-only controller for manual flight

The following further modifications to 'controller_mellinger.c' were made for this controller:
* Integral terms REMOVED
* L1 Adaptive Control augmentation added
*/

#include <math.h>
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "physicalConstants.h"
#include "stabilizer.h"

// physical parameters
static const float g_vehicleMass = CF_MASS;
static const float massThrust = 132000;
static const float moment_arm = 0.0326;             // perpendicular distance from propellors to body axes, in meters (m)

static struct mat33 J =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};

static struct mat33 Jinv =
    {{{0.061e6, -0.0029e6, -0.0013e6},
      {-0.0029e6, 0.061e6, -0.0037e6},
      {-0.0013e6, -0.0037e6, 0.034e6}}};


// geometric controller gains
static float kp_x = 0.4;
static float kp_y = 0.4;
static float kp_z = 1.25;

static float kv_x = 0.2;
static float kv_y = 0.2;
static float kv_z = 0.4;

static float kr_x = 70000;
static float kr_y = 70000;
static float kr_z = 60000;

static float ko_x = 20000;
static float ko_y = 20000;
static float ko_z = 12000;

static float kd_omega_rp = 200;


// state variables
static struct vec stateOmegaPrev = {0.0f, 0.0f, 0.0f};
static struct vec stateVelPrev = {0.0f, 0.0f, 0.0f};
static float stateOmegaPrev_roll = 0.0f;
static float stateOmegaPrev_pitch = 0.0f;
static float setpointOmegaPrev_roll = 0.0f;
static float setpointOmegaPrev_pitch = 0.0f;
static struct mat33 Rprev = 
    {{{0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f}}};

// state predictor
static struct vec v_tilda;
static struct vec w_tilda;
static struct vec v_hat;
static struct vec w_hat;
static struct vec v_hat_prev;
static struct vec w_hat_prev;

// adaptation law
static const struct vec As_v = {-5, -5, -5};
static const struct vec As_w = {-10, -10, -10};
static struct vec sigma_f;
static struct vec sigma_m;

// low pass filter
static float w_f1 =  1.0f;
static float w_f2 =  1.0f;
static float w_m1 = 10.0f;
static float w_m2 =  1.0f;

static float lpf_f1_coef1;
static float lpf_f1_coef2;
static float lpf_f2_coef1;
static float lpf_f2_coef2;
static float lpf_m1_coef1;
static float lpf_m1_coef2;
static float lpf_m2_coef1;
static float lpf_m2_coef2;

static float sigma_fz_filt1;
static float sigma_fz_filt2;
static struct vec sigma_m_filt1;
static struct vec sigma_m_filt2;
static float sigma_fz_filt1_prev;
static float sigma_fz_filt2_prev;
static struct vec sigma_m_filt1_prev;
static struct vec sigma_m_filt2_prev;

// PWM to SI conversion
static float thrustToTorque = 0.005964552f;

// logging variables
static struct vec z_axis_desired;

static float bsln_pwm_thrust;
static float bsln_SI_thrust;
static float adj_SI_thrust;
static float bsln_SI_thrust_prev;
static float adj_SI_thrust_prev;

static struct vec bsln_pwm_moments;
static struct vec bsln_SI_moments;
static struct vec adj_SI_moments;
static struct vec bsln_SI_moments_prev;
static struct vec adj_SI_moments_prev;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;

static float cmd_SI_thrust;
static struct vec cmd_SI_moments;

static float cmd_thrust_prev;
static float cmd_roll_prev;
static float cmd_pitch_prev;
static float cmd_yaw_prev;

void controllerL1Reset(void)
{
  // reset state predictor
  v_hat = mkvec(0.0f, 0.0f, 0.0f);
  w_hat = mkvec(0.0f, 0.0f, 0.0f);
  v_tilda = mkvec(0.0f, 0.0f, 0.0f);
  w_tilda = mkvec(0.0f, 0.0f, 0.0f);
  v_hat_prev = mkvec(0.0f, 0.0f, 0.0f);
  w_hat_prev = mkvec(0.0f, 0.0f, 0.0f);

  // reset adaptation law
  sigma_f = mkvec(0.0f, 0.0f, 0.0f);
  sigma_m = mkvec(0.0f, 0.0f, 0.0f);

  // reset LPF
  sigma_fz_filt1 = 0.0f;
  sigma_fz_filt2 = 0.0f;
  sigma_m_filt1 = mkvec(0.0f, 0.0f, 0.0f);
  sigma_m_filt2 = mkvec(0.0f, 0.0f, 0.0f);
  sigma_fz_filt1_prev = 0.0f;
  sigma_fz_filt2_prev = 0.0f;
  sigma_m_filt1_prev = mkvec(0.0f, 0.0f, 0.0f);
  sigma_m_filt2_prev = mkvec(0.0f, 0.0f, 0.0f);

  // reset adjusted inputs
  adj_SI_thrust = 0.0f;
  adj_SI_moments = mkvec(0.0f, 0.0f, 0.0f);
  adj_SI_thrust_prev = 0.0f;
  adj_SI_moments_prev = mkvec(0.0f, 0.0f, 0.0f);
  
}

void controllerL1Init(void)
{
  controllerL1Reset();
}

bool controllerL1Test(void)
{
  return true;
}


void bslnPWMtoSI(void)
{
  // convert u_b in PWM to motor power commands in PWM
  float t = bsln_pwm_thrust;
  float r = bsln_pwm_moments.x;
  float p = bsln_pwm_moments.y;
  float y = bsln_pwm_moments.z;

  float bsln_m1_PWM = t - r - p - y;
  float bsln_m2_PWM = t - r + p + y;
  float bsln_m3_PWM = t + r + p - y;
  float bsln_m4_PWM = t + r - p + y;

  float bsln_m1_SI = (2.130295e-11f)*(bsln_m1_PWM)*(bsln_m1_PWM) + (1.032633e-6f)*(bsln_m1_PWM) + (5.484560e-4f);
  float bsln_m2_SI = (2.130295e-11f)*(bsln_m2_PWM)*(bsln_m2_PWM) + (1.032633e-6f)*(bsln_m2_PWM) + (5.484560e-4f);
  float bsln_m3_SI = (2.130295e-11f)*(bsln_m3_PWM)*(bsln_m3_PWM) + (1.032633e-6f)*(bsln_m3_PWM) + (5.484560e-4f);
  float bsln_m4_SI = (2.130295e-11f)*(bsln_m4_PWM)*(bsln_m4_PWM) + (1.032633e-6f)*(bsln_m4_PWM) + (5.484560e-4f);

  // convert motor power commands in SI to u_b in SI
  bsln_SI_thrust =    ( bsln_m1_SI + bsln_m2_SI + bsln_m3_SI + bsln_m4_SI);
  bsln_SI_moments.x = (-bsln_m1_SI - bsln_m2_SI + bsln_m3_SI + bsln_m4_SI)*(moment_arm);
  bsln_SI_moments.y = (-bsln_m1_SI + bsln_m2_SI + bsln_m3_SI - bsln_m4_SI)*(moment_arm);
  bsln_SI_moments.z = (-bsln_m1_SI + bsln_m2_SI - bsln_m3_SI + bsln_m4_SI)*(thrustToTorque);

}

void cmdSItoPWM(void)
{
  // convert u_adj in SI to motor power commands in SI
  float t = cmd_SI_thrust    * (0.25f);
  float r = cmd_SI_moments.x * (0.25f / moment_arm);
  float p = cmd_SI_moments.y * (0.25f / moment_arm);
  float y = cmd_SI_moments.z * (0.25f / thrustToTorque);

  float cmd_SI_m1 = (t - r - p - y);
  float cmd_SI_m2 = (t - r + p + y);
  float cmd_SI_m3 = (t + r + p - y);
  float cmd_SI_m4 = (t + r - p + y);

  float cmd_m1_pwm = ((-1.032633e-6f) + sqrtf((1.032633e-6f)*(1.032633e-6f) - 4*(2.130295e-11f)*(5.484560e-4f - cmd_SI_m1))) / (2.0f*(2.130295e-11f));
  float cmd_m2_pwm = ((-1.032633e-6f) + sqrtf((1.032633e-6f)*(1.032633e-6f) - 4*(2.130295e-11f)*(5.484560e-4f - cmd_SI_m2))) / (2.0f*(2.130295e-11f));
  float cmd_m3_pwm = ((-1.032633e-6f) + sqrtf((1.032633e-6f)*(1.032633e-6f) - 4*(2.130295e-11f)*(5.484560e-4f - cmd_SI_m3))) / (2.0f*(2.130295e-11f));
  float cmd_m4_pwm = ((-1.032633e-6f) + sqrtf((1.032633e-6f)*(1.032633e-6f) - 4*(2.130295e-11f)*(5.484560e-4f - cmd_SI_m4))) / (2.0f*(2.130295e-11f));

  cmd_thrust = ( cmd_m1_pwm + cmd_m2_pwm + cmd_m3_pwm + cmd_m4_pwm) * (0.25f);
  cmd_roll   = (-cmd_m1_pwm - cmd_m2_pwm + cmd_m3_pwm + cmd_m4_pwm) * (0.25f);
  cmd_pitch  = (-cmd_m1_pwm + cmd_m2_pwm + cmd_m3_pwm - cmd_m4_pwm) * (0.25f);
  cmd_yaw    = (-cmd_m1_pwm + cmd_m2_pwm - cmd_m3_pwm + cmd_m4_pwm) * (0.25f);
}


void L1Augmentation(struct mat33 currentR, struct vec currentVel, struct vec currentOmega, float dt) 
{

  // body axes
  struct vec xB = mcolumn(Rprev, 0);
  struct vec yB = mcolumn(Rprev, 1);
  struct vec zB = mcolumn(Rprev, 2);

// ------------------ PREDICTOR ------------------

  // compute Z_f and Z_m -> vectors that combine the baseline control, adjusted control, and uncertainty
  struct vec Z_f = mkvec(
    sigma_f.x, 
    sigma_f.y, 
    sigma_f.z + bsln_SI_thrust_prev + adj_SI_thrust_prev
  );
  struct vec Z_m = vadd3(bsln_SI_moments_prev, adj_SI_moments_prev, sigma_m);

  // calculate v_hat_dot
  struct vec v_hat_dot = vadd4(
    mkvec(0.0, 0.0, -GRAVITY_MAGNITUDE), 
    vscl((1/g_vehicleMass) * Z_f.z, zB), 
    vadd(vscl((1/g_vehicleMass)*Z_f.x, xB), vscl((1/g_vehicleMass)*Z_f.y, yB)), 
    veltmul(As_v, v_tilda)
  );

  // calculate w_hat_dot
  struct vec w_hat_dot = vadd3(
    vcross(mvmul(mneg(Jinv), stateOmegaPrev), mvmul(J, stateOmegaPrev)), 
    mvmul(Jinv, Z_m), 
    veltmul(As_w, w_tilda)
  );

  // use forward euler to calculate next z_hat
  v_hat = vadd(v_hat_prev, vscl(dt, v_hat_dot));
  w_hat = vadd(w_hat_prev, vscl(dt, w_hat_dot));

  // compute prediction error
  v_tilda = vsub(v_hat, currentVel);
  w_tilda = vsub(w_hat, currentOmega);

  // ------------------ ADAPTATION LAW ------------------

  // exponential coefficients
  struct vec exp_As_dt_v = mkvec(
    expf(As_v.x * dt), 
    expf(As_v.y * dt), 
    expf(As_v.z * dt)
  );
  struct vec exp_As_dt_w = mkvec(
    expf(As_w.x * dt), 
    expf(As_w.y * dt), 
    expf(As_w.z * dt)
  );

  // later part of uncertainty estimation (piecewise constant)
  struct vec PhiInvmu_v = veltdiv(veltmul(v_tilda, veltmul(As_v, exp_As_dt_v)), vsub(exp_As_dt_v, mkvec(1,1,1)));
  struct vec PhiInvmu_w = veltdiv(veltmul(w_tilda, veltmul(As_w, exp_As_dt_w)), vsub(exp_As_dt_w, mkvec(1,1,1)));

  // uncertainties
  sigma_f = vscl(-g_vehicleMass, mvmul(mtranspose(Rprev), PhiInvmu_v));
  sigma_m = vneg(mvmul(J, PhiInvmu_w));

  // ------------------ LOW PASS FILTER ------------------

  // filter SI measurements
  lpf_f1_coef1 = expf(-w_f1*dt);
  lpf_f1_coef2 = 1.0f - lpf_f1_coef1;
  lpf_f2_coef1 = expf(-w_f2*dt);
  lpf_f2_coef2 = 1.0f - lpf_f2_coef1;
  lpf_m1_coef1 = expf(-w_m1*dt);
  lpf_m1_coef2 = 1.0f - lpf_m1_coef1;
  lpf_m2_coef1 = expf(-w_m2*dt);
  lpf_m2_coef2 = 1.0f - lpf_m2_coef1;

  sigma_fz_filt1 =  (lpf_f1_coef1  * sigma_fz_filt1_prev)  + (lpf_f1_coef2  * sigma_f.z);
  sigma_fz_filt2 =  (lpf_f2_coef2  * sigma_fz_filt2_prev) + (lpf_f2_coef2  * sigma_fz_filt1);
  sigma_m_filt1 = vadd(vscl(lpf_m1_coef1, sigma_m_filt1_prev), vscl(lpf_m1_coef2, sigma_m));
  sigma_m_filt2 = vadd(vscl(lpf_m2_coef1, sigma_m_filt2_prev), vscl(lpf_m2_coef2, sigma_m_filt1));

  adj_SI_thrust = -sigma_fz_filt1;
  adj_SI_moments = mkvec(-sigma_m_filt2.x, -sigma_m_filt2.y, -sigma_m_filt2.z);


  // iterate

  // state prediction
  v_hat_prev = v_hat;
  w_hat_prev = w_hat;

  // LPF
  sigma_fz_filt1_prev = sigma_fz_filt1;
  sigma_fz_filt2_prev = sigma_fz_filt2;
  sigma_m_filt1_prev = sigma_m_filt1;
  sigma_m_filt2_prev = sigma_m_filt2;

}



void controllerL1(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  struct vec r_error;
  struct vec v_error;
  struct vec target_thrust;
  float current_thrust;
  struct vec z_axis;
  struct vec x_axis_desired;
  struct vec y_axis_desired;
  struct vec x_c_des;
  struct vec eR, ew;
  struct vec M;
  float dt;
  float desiredYaw = 0; //deg

  control->controlMode = controlModeLegacy;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
    return;
  }

  dt = (float)(1.0f/ATTITUDE_RATE);
  struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
  struct vec setpointVel = mkvec(setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z);
  struct vec statePos = mkvec(state->position.x, state->position.y, state->position.z);
  struct vec stateVel = mkvec(state->velocity.x, state->velocity.y, state->velocity.z);

  // Position Error (ep)
  r_error = vsub(setpointPos, statePos);

  // Velocity Error (ev)
  v_error = vsub(setpointVel, stateVel);

  // Desired thrust [F_des]
  if (setpoint->mode.x == modeAbs) {
    target_thrust.x = g_vehicleMass * setpoint->acceleration.x                       + kp_x * r_error.x + kv_x * v_error.x;
    target_thrust.y = g_vehicleMass * setpoint->acceleration.y                       + kp_y * r_error.y + kv_y * v_error.y;
    target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAVITY_MAGNITUDE) + kp_z * r_error.z + kv_z * v_error.z;

  } else {
    target_thrust.x = -sinf(radians(setpoint->attitude.pitch));
    target_thrust.y = -sinf(radians(setpoint->attitude.roll));
    // In case of a timeout, the commander tries to level, ie. x/y are disabled, but z will use the previous setting
    // In that case we ignore the last feedforward term for acceleration

    if (setpoint->mode.z == modeAbs) {
      target_thrust.z = g_vehicleMass * GRAVITY_MAGNITUDE + kp_z * r_error.z + kv_z * v_error.z;

    } else {
      target_thrust.z = 1;
    }
  }

  // Rate-controlled YAW is moving YAW angle setpoint
  if (setpoint->mode.yaw == modeVelocity) {
    desiredYaw = state->attitude.yaw + setpoint->attitudeRate.yaw * dt;
  } else if (setpoint->mode.yaw == modeAbs) {
    desiredYaw = setpoint->attitude.yaw;
  } else if (setpoint->mode.quat == modeAbs) {
    struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    desiredYaw = degrees(rpy.z);
  }

  // Z-Axis [zB]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  z_axis = mcolumn(R, 2);

  // yaw correction (only if position control is not used)
  if (setpoint->mode.x != modeAbs) {
    struct vec x_yaw = mcolumn(R, 0);
    x_yaw.z = 0;
    x_yaw = vnormalize(x_yaw);
    struct vec y_yaw = vcross(mkvec(0, 0, 1), x_yaw);
    struct mat33 R_yaw_only = mcolumns(x_yaw, y_yaw, mkvec(0, 0, 1));
    target_thrust = mvmul(R_yaw_only, target_thrust);
  }

  // Current thrust [F]
  current_thrust = vdot(target_thrust, z_axis);

  // Calculate axis [zB_des]
  z_axis_desired = vnormalize(target_thrust);

  // [xC_des]
  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  x_c_des.x = cosf(radians(desiredYaw));
  x_c_des.y = sinf(radians(desiredYaw));
  x_c_des.z = 0;
  // [yB_des]
  y_axis_desired = vnormalize(vcross(z_axis_desired, x_c_des));
  // [xB_des]
  x_axis_desired = vcross(y_axis_desired, z_axis_desired);

  // [eR]
  // Slow version
  // struct mat33 Rdes = mcolumns(
  //   mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
  //   mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
  //   mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));

  // struct mat33 R_transpose = mtranspose(R);
  // struct mat33 Rdes_transpose = mtranspose(Rdes);

  // struct mat33 eRM = msub(mmult(Rdes_transpose, R), mmult(R_transpose, Rdes));

  // eR.x = eRM.m[2][1];
  // eR.y = -eRM.m[0][2];
  // eR.z = eRM.m[1][0];

  // Fast version (generated using Mathematica)
  float x = q.x;
  float y = q.y;
  float z = q.z;
  float w = q.w;
  eR.x = (-1 + 2*fsqr(x) + 2*fsqr(y))*y_axis_desired.z + z_axis_desired.y - 2*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*z_axis_desired.x + fsqr(x)*z_axis_desired.y + fsqr(z)*z_axis_desired.y - y*z*z_axis_desired.z) +    2*w*(-(y*y_axis_desired.x) - z*z_axis_desired.x + x*(y_axis_desired.y + z_axis_desired.z));
  eR.y = x_axis_desired.z - z_axis_desired.x - 2*(fsqr(x)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (fsqr(y) + fsqr(z))*z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*z_axis_desired.y + z*z_axis_desired.z) + w*(x*x_axis_desired.y + z*z_axis_desired.y - y*(x_axis_desired.x + z_axis_desired.z)));
  eR.z = y_axis_desired.x - 2*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2*y_axis_desired.x*fsqr(z) + x_axis_desired.y*(-1 + 2*fsqr(x) + 2*fsqr(z));

  // Account for Crazyflie coordinate system
  // eR.y = -eR.y;          // change here (original had this line)

  // [ew]
  // ew = omega - Rtrans*Rdes*Omegades
  float err_d_roll = 0;
  float err_d_pitch = 0;

  float stateAttitudeRateRoll = radians(sensors->gyro.x);
  // float stateAttitudeRatePitch = -radians(sensors->gyro.y);   // change here (original had the negative)
  float stateAttitudeRatePitch = radians(sensors->gyro.y);
  float stateAttitudeRateYaw = radians(sensors->gyro.z);

  struct vec stateOmega = mkvec(stateAttitudeRateRoll, -stateAttitudeRatePitch, stateAttitudeRateYaw);

  float setpointAttitudeRateRoll = radians(setpoint->attitudeRate.roll);
  // float setpointAttitudeRatePitch = -radians(setpoint->attitudeRate.pitch); // change here (original had the negative)
  float setpointAttitudeRatePitch = radians(setpoint->attitudeRate.pitch);
  float setpointAttitudeRateYaw = radians(setpoint->attitudeRate.yaw);

  ew.x = setpointAttitudeRateRoll - stateAttitudeRateRoll;
  ew.y = setpointAttitudeRatePitch - stateAttitudeRatePitch;
  ew.z = setpointAttitudeRateYaw - stateAttitudeRateYaw;
  if (stateOmegaPrev_roll == stateOmegaPrev_roll) { /* d part initialized */
    err_d_roll = ((radians(setpoint->attitudeRate.roll) - setpointOmegaPrev_roll) - (stateAttitudeRateRoll - stateOmegaPrev_roll)) / dt;
    // err_d_pitch = (-(radians(setpoint->attitudeRate.pitch) - setpointOmegaPrev_pitch) - (stateAttitudeRatePitch - stateOmegaPrev_pitch)) / dt;   // change here (only the first term was originally negative) 
    err_d_pitch = ((radians(setpoint->attitudeRate.pitch) - setpointOmegaPrev_pitch) - (stateAttitudeRatePitch - stateOmegaPrev_pitch)) / dt;
  }
  stateOmegaPrev_roll = stateAttitudeRateRoll;
  stateOmegaPrev_pitch = stateAttitudeRatePitch;
  setpointOmegaPrev_roll = radians(setpoint->attitudeRate.roll);
  setpointOmegaPrev_pitch = radians(setpoint->attitudeRate.pitch);

  // Moment:
  M.x = -kr_x * eR.x + ko_x * ew.x + kd_omega_rp * err_d_roll;
  M.y = -kr_y * eR.y + ko_y * ew.y + kd_omega_rp * err_d_pitch;
  M.z = -kr_z * eR.z + ko_z * ew.z;

  // calculate baseline force
  if (setpoint->mode.z == modeDisable) {
    bsln_pwm_thrust = setpoint->thrust;
  } else {
    bsln_pwm_thrust = current_thrust * massThrust;
  }

  // r_roll = radians(sensors->gyro.x);
  // r_pitch = -radians(sensors->gyro.y);
  // r_yaw = radians(sensors->gyro.z);

  // calculate baseline moments
  if (bsln_pwm_thrust > 0) {
    bsln_pwm_moments.x = clamp(M.x, -32000, 32000);
    bsln_pwm_moments.y = clamp(M.y, -32000, 32000);
    bsln_pwm_moments.z = clamp(-M.z, -32000, 32000);

  } else {
    bsln_pwm_moments.x = 0;
    bsln_pwm_moments.y = 0;
    bsln_pwm_moments.z = 0;

    controllerL1Reset();
  }


  // convert baseline control inputs into standard units
  bslnPWMtoSI();

  // do L1, but try to avoid running L1 if the drone is sitting on the ground
  if (cmd_thrust > 0){
    L1Augmentation(R, stateVel, stateOmega, dt);
  }

  // compute total control inputs (baseline + adjusted) in standard units
  cmd_SI_thrust = bsln_SI_thrust + adj_SI_thrust;
  cmd_SI_moments = vadd(bsln_SI_moments, adj_SI_moments);  

  // convert total control inputs back into CF units, from standard units
  cmdSItoPWM();

  // apply control inputs
  control->thrust = cmd_thrust;
  control->roll = cmd_roll;
  control->pitch = -cmd_pitch;    // the negative here is to deal with the crazyflie's internal coordinate system
  control->yaw = cmd_yaw;

  // iterate
  stateVelPrev = stateVel;
  stateOmegaPrev = stateOmega;
  Rprev = R;

  bsln_SI_thrust_prev = bsln_SI_thrust;
  bsln_SI_moments_prev = bsln_SI_moments;
  adj_SI_thrust_prev = adj_SI_thrust;
  adj_SI_moments_prev = adj_SI_moments;

  cmd_thrust_prev = cmd_thrust;
  cmd_roll_prev = cmd_roll;
  cmd_pitch_prev = cmd_pitch;
  cmd_yaw_prev = cmd_yaw;
}




// Tuning variables for the L1 Adaptive Controller
PARAM_GROUP_START(ctrlL1)

PARAM_ADD(PARAM_FLOAT, kp_x, &kp_x)                 // P-gain for x-position
PARAM_ADD(PARAM_FLOAT, kp_y, &kp_y)                 // P-gain for y-position
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)                 // P-gain for z-position
PARAM_ADD(PARAM_FLOAT, kv_x, &kv_x)                 // D-gain for x-position
PARAM_ADD(PARAM_FLOAT, kv_y, &kv_y)                 // D-gain for y-position
PARAM_ADD(PARAM_FLOAT, kv_z, &kv_z)                 // D-gain for z-position
PARAM_ADD(PARAM_FLOAT, kr_x, &kr_x)                 // P-gain for roll angle
PARAM_ADD(PARAM_FLOAT, kr_y, &kr_y)                 // P-gain for pitch angle
PARAM_ADD(PARAM_FLOAT, kr_z, &kr_z)                 // P-gain for yaw angle
PARAM_ADD(PARAM_FLOAT, ko_x, &ko_x)                 // D-gain for pitch angle
PARAM_ADD(PARAM_FLOAT, ko_y, &ko_y)                 // D-gain for roll angle
PARAM_ADD(PARAM_FLOAT, ko_z, &ko_z)                 // D-gain for yaw angle
PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)   // D-gain for roll and pitch angular velocity

PARAM_ADD(PARAM_FLOAT, w_f1, &w_f1)                 // LPF cutoff frequency, forces (1/2)
PARAM_ADD(PARAM_FLOAT, w_f2, &w_f2)                 // LPF cutoff frequency, forces (2/2)
PARAM_ADD(PARAM_FLOAT, w_m1, &w_m1)                 // LPF cutoff frequency, moments (1/2)
PARAM_ADD(PARAM_FLOAT, w_m2, &w_m2)                 // LPF cutoff frequency, moments (1/2)

PARAM_GROUP_STOP(ctrlL1)


// Logging variables for the L1 Adaptive Controller
LOG_GROUP_START(ctrlL1)

LOG_ADD(LOG_FLOAT, bsln_thrust, &bsln_pwm_thrust)   // baseline thrust command
LOG_ADD(LOG_FLOAT, bsln_roll, &bsln_pwm_moments.x)  // baseline roll command
LOG_ADD(LOG_FLOAT, bsln_pitch, &bsln_pwm_moments.y) // baseline pitch command
LOG_ADD(LOG_FLOAT, bsln_yaw, &bsln_pwm_moments.z)   // baseline yaw command

LOG_ADD(LOG_FLOAT, unc_fx, &sigma_f.x)              // uncertainty estimate, x-component force (Newtons)
LOG_ADD(LOG_FLOAT, unc_fy, &sigma_f.y)              // uncertainty estimate, y-component force (Newtons)
LOG_ADD(LOG_FLOAT, unc_fz, &sigma_f.z)              // uncertainty estimate, z-component force (Newtons)
LOG_ADD(LOG_FLOAT, unc_Mx, &sigma_m.x)              // uncertainty estimate, x-component moment (Newtons)
LOG_ADD(LOG_FLOAT, unc_My, &sigma_m.y)              // uncertainty estimate, y-component moment (Newtons)
LOG_ADD(LOG_FLOAT, unc_Mz, &sigma_m.z)              // uncertainty estimate, z-component moment (Newtons)

LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)         // total thrust command
LOG_ADD(LOG_FLOAT, cmd_Mx, &cmd_roll)               // total roll command
LOG_ADD(LOG_FLOAT, cmd_My, &cmd_pitch)              // total pitch command
LOG_ADD(LOG_FLOAT, cmd_Mz, &cmd_yaw)                // total yaw command

LOG_GROUP_STOP(ctrlL1)