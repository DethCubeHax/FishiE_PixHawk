//
// Created by Lawrence Chen on 2021/4/17.
//

#ifndef PX4_SWASHPLATELESS_HPP
#define PX4_SWASHPLATELESS_HPP

//sensor parameters, need to change when switch experimental platform
// On single_axis vehicle
#define SENSOR_ROTOR_ANGLE_BIAS  (227.7)  //deg
#define SENDOR_PWM_MAX           (9658)
#define SENDOR_PWM_MIN           (292)
// End

// On F-T test bench
//#define SENSOR_ROTOR_ANGLE_BIAS  (149.27)  //deg
//#define SENDOR_PWM_MAX           (9366)
//#define SENDOR_PWM_MIN           (284)
// End

//#define MOTOR_DELAY_ANLGE_BIAS (2.496) //rad, throttle = 1050
//#define MOTOR_DELAY_ANLGE_BIAS (3.09) //2.496 + 0.594 = 3.09 rad, throttle = 1436, thrust = 11.8N
#define MOTOR_DELAY_ANLGE_BIAS   (2.88)  //165deg, 1200-1300 dshot
//#define MOTOR_DELAY_ANLGE_BIAS (0.0)


#define SWASH_SIN_DEADZONE (10)

typedef struct
{
    uint64_t timestamp;
    float pulse_angle_deg;
    float pulse_angle_deg_last;
    float pulse_angle_deg_delta;
    float pulse_speed_rpm;
    float throttle_sin;
    float dt;
    uint16_t throttle_out;
    float ref_x;
    float ref_y;
    float sin_amp;
    float sin_pha;
}Motor_State_Def;

typedef struct
{
    uint32_t pulse_width;
    uint32_t pulse_width_last;
    uint32_t period;
    uint64_t timestamp_curr;
    uint64_t timestamp_last;
    float dt;
    uint32_t width_max;
    uint32_t width_min;
    uint32_t width_range;
}PWM_CAP_Def;

typedef struct
{
    float LX;
    float LY;
    float RX;
    float RY;

    float LX_last;
    float LY_last;
    float RX_last;
    float RY_last;

    int8_t switch_B;
    int8_t switch_C;
    int8_t switch_D;
    int8_t switch_E;
    int8_t switch_G;

    int8_t switch_B_last;
    int8_t switch_C_last;
    int8_t switch_D_last;
    int8_t switch_E_last;
    int8_t switch_G_last;

    float L_amp;
    float L_pha_rad;
    float R_amp;
    float R_pha_rad;
}RC_Def;

typedef struct
{
    uint64_t yaw_rate_timestamp;
    uint64_t yaw_rate_timestamp_last;
    float yaw_rate_dt;

    float vehicle_yaw_rad;
    float vehicle_yaw_rate_radps;

    float body_yaw_rad;
    float body_yaw_rate_radps;

    float actuator_ctrls_roll;
    float actuator_ctrls_pitch;
    float actuator_ctrls_yaw;
    float actuator_ctrls_thrust;
    float actuator_ctrls_amp;
    float actuator_ctrls_pha;

    float bat_vol_now;
    float bat_vol_compen_coeff;
}SINGLE_AXIS_PROP_Def;

float motor_angle_cal(uint32_t pwm_now, uint32_t pwm_max, uint32_t pwm_min, float angle_bias);
uint16_t speed_control_for_dshot(bool on_flag);


#endif //PX4_SWASHPLATELESS_HPP
