/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_log.h>
#include <string.h> //for include memset
#include <poll.h>

#include <matrix/matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/orb_motor_telemetry_me.h>
#include <uORB/topics/orb_motor_status_me.h>

#include <uORB/topics/adc_report.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/key_command.h>
#include <uORB/topics/pwm_captures.h>
#include <uORB/topics/pwm_encoder_me.h>

#include "px4_cn.h"

#define ROTOR_ANGLE_BIAS 97.6

//#define PRINT_RC
#define PRINT_PWM_IN
//#define PRINT_MOTOR_STATUS
//#define PRINT_DEBUG_VALUE
//#define PRINT_ACTUATOR_CONTROL
//#define PRINT_VEHICLE_CONTROL
//#define PRINT_SENSOR_MAG
//#define PRINT_SENSOR_ACCEL
//#define PRINT_KEY_COMMAND

int uORB_data_print_old();
int uORB_data_print_new();

extern "C" __EXPORT int px4_cn_main(int argc, char *argv[])
{
    PX4_INFO("px4_cn_main enter");
//    uORB_data_print_old();
//    uORB_data_print_new();

//    px4_task_spawn_cmd("px4_cn_thread",
//                       SCHED_DEFAULT,
//                       SCHED_PRIORITY_DEFAULT,
//                       1024,
//                       (px4_main_t) uORB_data_print_new,
//                       nullptr);
//    return 0;

    return PX4cn::main(argc, argv);
}

int uORB_data_print_new()
{
    static SUBSCRIBE_VAR_t subscribe_var{0};

    // subscribe adc_data from adc hardware
    struct adc_report_s adc_data{0};
    uORB::Subscription adc_report_sub{ORB_ID(adc_report)};

    // subscribe pwm_data from pwm_capture
    struct pwm_input_s pwm_input_data{0};
    uORB::Subscription pwm_input_sub{ORB_ID(pwm_input)};

    // subscribe rc_data from remote controller
    struct rc_channels_s rc_data{0};
    uORB::Subscription rc_channels_sub{ORB_ID(rc_channels)};

    // subscribe motor_status from dshot thread
    struct orb_motor_status_me_s motor_status_data{0};
    uORB::Subscription orb_motor_status_me_sub{ORB_ID(orb_motor_status_me)};

    // subscribe debug_value
    struct debug_value_s debug_value_data{0};
    uORB::Subscription debug_value_sub{ORB_ID(debug_value)};

    // subscribe actuator_controls
    struct actuator_controls_s actuator_controls_data{0};
    uORB::Subscription actuator_controls_sub{ORB_ID(actuator_controls_0)};
//    actuator_controls_s actuator_controls_data[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
//    uORB::SubscriptionCallbackWorkItem actuator_controls_sub[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

    PX4_INFO("px4_cn is running");

    while(true)
    {
#ifdef PRINT_PWM_IN
        if(pwm_input_sub.update(&pwm_input_data))
        {
            subscribe_var.pwm_pulse_width = pwm_input_data.pulse_width;
            subscribe_var.pwm_period = pwm_input_data.period;
            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
            if(angle > 360)
                angle = 360;
            else if(angle < 0)
                angle = 0;
            angle -= ROTOR_ANGLE_BIAS;
            if(angle < 0) angle += 360;
            angle = 360 - angle;
//            PX4_INFO("width = %d, period = %d, angle = %f", subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle);
            PX4_INFO("width = %d, conter = %d, angle = %f", subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle);
        }
#endif

#ifdef PRINT_RC
        if(rc_channels_sub.update(&rc_data))
        {
            subscribe_var.rc_LX = (double)rc_data.channels[3];
            subscribe_var.rc_LY = (double)rc_data.channels[1];
            subscribe_var.rc_RX = (double)rc_data.channels[0];
            subscribe_var.rc_RY = (double)rc_data.channels[2];
//             PX4_INFO("rc = %f %f %f %f", subscribe_var.rc_LX, subscribe_var.rc_LY, subscribe_var.rc_RX, subscribe_var.rc_RY);
            PX4_INFO("0=%.3f, 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f,"
                     "7=%.3f, 8=%.3f, 9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f, 13=%.3f",
                     rc_data.channels[0], rc_data.channels[1],
                     rc_data.channels[2], rc_data.channels[3],
                     rc_data.channels[4], rc_data.channels[5],
                     rc_data.channels[6], rc_data.channels[7],
                     rc_data.channels[8], rc_data.channels[9],
                     rc_data.channels[10], rc_data.channels[11],
                     rc_data.channels[12], rc_data.channels[13]
            );
        }
#endif

#ifdef PRINT_MOTOR_STATUS
        if(orb_motor_status_me_sub.update(&motor_status_data))
        {
            static uint32_t counter = 0;
            counter++;
            subscribe_var.motor_angle_deg = motor_status_data.angle_deg[4];
            subscribe_var.motor_speed_rpm = motor_status_data.speed_rpm[4];
            subscribe_var.dt_s = motor_status_data.dt[4];
            if(counter > 10)
            {
                PX4_INFO("deg = %.3f, rpm = %.3f, dt = %.3f, cnt = %d, out = %d, "
                         "sin = %.3f, amp = %.3f, pha = %.3f",
                         subscribe_var.motor_angle_deg, subscribe_var.motor_speed_rpm, subscribe_var.dt_s,
                         motor_status_data.frame_counter, motor_status_data.throttle_out[4],
                         motor_status_data.throttle_sin[4], motor_status_data.sin_amp[4],
                         motor_status_data.sin_pha[4]);
                counter = 0;
            }
        }
#endif

#ifdef PRINT_DEBUG_VALUE
        if(debug_value_sub.update(&debug_value_data))
        {
            PX4_INFO("debug: ind = %d, value = %.3f", debug_value_data.ind, debug_value_data.value);
        }
#endif

#ifdef PRINT_ACTUATOR_CONTROL
        if(actuator_controls_sub.update(&actuator_controls_data))
        {
            PX4_INFO("actuator_controls = %.3f, %.3f, %.3f, %.3f",
            actuator_controls_data.control[0],
            actuator_controls_data.control[1],
            actuator_controls_data.control[2],
            actuator_controls_data.control[3]);
        }
#endif

//        PX4_INFO("PX4_INFO");
//        sleep(1);//parameter is int
          usleep(100000);
    }
}



int uORB_data_print_old()
{
    static struct timespec ts;
    static SUBSCRIBE_VAR_t subscribe_var{0};
    static uint32_t counter = 0;

    px4_clock_gettime(CLOCK_REALTIME, &ts);
    RunTimeCalculation time_cal(ts);

    // subscribe motor_data from telemetry
    struct orb_motor_telemetry_me_s motor_data;
	memset(&motor_data, 0, sizeof(motor_data));
    int motor_data_fd = orb_subscribe(ORB_ID(orb_motor_telemetry_me));

    // subscribe motor_status from dshot thread
    struct orb_motor_status_me_s motor_status;
    memset(&motor_status, 0, sizeof(motor_status));
    int motor_status_fd = orb_subscribe(ORB_ID(orb_motor_status_me));

    // // subscribe adc_data from adc hardware
    struct adc_report_s adc_data;
	memset(&adc_data, 0, sizeof(adc_data));
    int adc_data_fd = orb_subscribe(ORB_ID(adc_report));

    // subscribe esc_data from esc
    struct esc_report_s esc_data;
    memset(&esc_data, 0, sizeof(esc_data));
    int esc_data_fd = orb_subscribe(ORB_ID(esc_report));

    // subscribe rc_data from remote controller
    struct rc_channels_s rc_data;
	memset(&rc_data, 0, sizeof(rc_data));
    int rc_data_fd = orb_subscribe(ORB_ID(rc_channels));

    // subscribe pwm_data from pwm_capture
    struct pwm_input_s pwm_input_data;
	memset(&pwm_input_data, 0, sizeof(pwm_input_data));
    int pwm_input_data_fd = orb_subscribe(ORB_ID(pwm_input));

    // create poll fd
    struct pollfd fds[] = {
         { .fd = motor_status_fd,    .events = POLLIN },
//         { .fd = motor_data_fd,      .events = POLLIN },
//         { .fd = adc_data_fd,        .events = POLLIN },
//         { .fd = esc_data_fd,        .events = POLLIN },
//         { .fd = rc_data_fd,         .events = POLLIN },
//         { .fd = pwm_input_data_fd,  .events = POLLIN },
    };

    PX4_INFO("px4_cn is running");

    
    while(true)
    {
        px4_clock_gettime(CLOCK_REALTIME, &ts);
        time_cal.update(ts);
//        int poll_ret = poll(fds, 1, 1000);

//        bool rc_check_flag;
//        double LX = 0.0, LY = 0.0, RX = 0.0, RY = 0.0;
//        orb_check(rc_data_fd, &rc_check_flag);
//        if(rc_check_flag == true)
//        {
//            orb_copy(ORB_ID(rc_channels), rc_data_fd, &rc_data);
//            LX = (double)rc_data.channels[3];
//            LY = (double)rc_data.channels[1];
//            RX = (double)rc_data.channels[0];
//            RY = (double)rc_data.channels[2]; //throttle, value: 0 ~ 1, in my rc
//            PX4_INFO("rc = %f %f %f %f", LX, LY, RX, RY);
//            uint16_t throttle_out = (uint16_t)(RY * 2047.0);
//            PX4_INFO("throttle_out = %d", throttle_out);
//        }
//        usleep(1000000);

//        if (fds[0].revents & POLLIN)
        if(0)
        {
            orb_copy(ORB_ID(orb_motor_telemetry_me), motor_data_fd, &motor_data);
            subscribe_var.motor_rpm = motor_data.erpm * 24;
//            PX4_INFO("RPM = %d, I = %d, U = %d, T = %d, P = %d\n",
//            motor_data.erpm * 14, motor_data.current, motor_data.voltage, motor_data.temperature, motor_data.consumption);
//            px4_clock_gettime(CLOCK_REALTIME, &ts);
//            time_cal.update(ts);
//            PX4_INFO("freq = %f， motor_report: RPM = %d", time_cal.freq, subscribe_var.motor_rpm);
        }

//        if (fds[1].revents & POLLIN)
        if(0)
        {
            orb_copy(ORB_ID(adc_report), adc_data_fd, &adc_data);
            subscribe_var.adc_V33 = (double)adc_data.raw_data[10] / 4095.0 * 3.30;
            subscribe_var.adc_V66 = (double)adc_data.raw_data[4]  / 4095.0 * 6.60;
//            PX4_INFO("adc_value = %f, %f, cnt = %d", subscribe_var.adc_V33, subscribe_var.adc_V66);
        }

//         if (fds[2].revents & POLLIN)
         if(0)
         {
             orb_copy(ORB_ID(esc_report), esc_data_fd, &esc_data);
             subscribe_var.esc_rpm = esc_data.esc_rpm;
//             PX4_INFO("esc_report: RPM = %d", subscribe_var.esc_rpm);
         }

//         if (fds[0].revents & POLLIN)
         if(0)
         {
             orb_copy(ORB_ID(rc_channels), rc_data_fd, &rc_data);
             subscribe_var.rc_LX = (double)rc_data.channels[3];
             subscribe_var.rc_LY = (double)rc_data.channels[1];
             subscribe_var.rc_RX = (double)rc_data.channels[0];
             subscribe_var.rc_RY = (double)rc_data.channels[2];
//             PX4_INFO("rc = %f %f %f %f", subscribe_var.rc_LX, subscribe_var.rc_LY, subscribe_var.rc_RX, subscribe_var.rc_RY);
             PX4_INFO("0=%.3f, 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f,"
                      "7=%.3f, 8=%.3f, 9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f, 13=%.3f",
                      rc_data.channels[0], rc_data.channels[1],
                      rc_data.channels[2], rc_data.channels[3],
                      rc_data.channels[4], rc_data.channels[5],
                      rc_data.channels[6], rc_data.channels[7],
                      rc_data.channels[8], rc_data.channels[9],
                      rc_data.channels[10], rc_data.channels[11],
                      rc_data.channels[12], rc_data.channels[13]
                      );
         }

        if (fds[0].revents & POLLIN)
        if(0)
        {
         orb_copy(ORB_ID(pwm_input), pwm_input_data_fd, &pwm_input_data);
         subscribe_var.pwm_pulse_width = pwm_input_data.pulse_width;
         subscribe_var.pwm_period = pwm_input_data.period;
         float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
         if(angle > 360)
             angle = 360;
         else if(angle < 0)
             angle = 0;
         angle -= ROTOR_ANGLE_BIAS;
         if(angle < 0) angle += 360;
         angle = 360 - angle;
         PX4_INFO("width = %d, period = %d, angle = %f",
                  subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle);
        }

//        if (fds[0].revents & POLLIN)
        if(0)
        {
            counter++;
            orb_copy(ORB_ID(orb_motor_status_me), motor_status_fd, &motor_status);
            subscribe_var.motor_angle_deg = motor_status.angle_deg[4];
            subscribe_var.motor_speed_rpm = motor_status.speed_rpm[4];
            subscribe_var.dt_s = motor_status.dt[4];
            if(counter > 10)
            {
                PX4_INFO("deg = %.3f, rpm = %.3f, dt = %.3f, cnt = %d, out = %d, "
                         "sin = %.3f, amp = %.3f, pha = %.3f",
                         subscribe_var.motor_angle_deg, subscribe_var.motor_speed_rpm, subscribe_var.dt_s,
                         motor_status.frame_counter, motor_status.throttle_out[4], motor_status.throttle_sin[4],
                         motor_status.sin_amp[4], motor_status.sin_pha[4]);
                counter = 0;
            }
        }
//        PX4_INFO("freq = %f, m_rpm = %d, adc = %f %f, rc = %f, width = %d, period = %d",
//                 time_cal.freq, subscribe_var.motor_rpm, subscribe_var.adc_V33, subscribe_var.adc_V66, subscribe_var.rc_RY,
//                 subscribe_var.pwm_pulse_width, subscribe_var.pwm_period);


    }
    return OK;
}




int PX4cn::print_status()
{
    PX4_INFO("Running");
    return 0;
}

int PX4cn::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}


int PX4cn::task_spawn(int argc, char *argv[])
{
    //stack too small (such as 1024) will cause crash when print orb_motor_status_me
    _task_id = px4_task_spawn_cmd("PX4cn",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

PX4cn *PX4cn::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
            case 'p':
                example_param = (int)strtol(myoptarg, nullptr, 10);
                break;

            case 'f':
                example_flag = true;
                break;

            case '?':
                error_flag = true;
                break;

            default:
                PX4_WARN("unrecognized flag");
                error_flag = true;
                break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    PX4cn *instance = new PX4cn(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

PX4cn::PX4cn(int example_param, bool example_flag)
        : ModuleParams(nullptr)
{}

void PX4cn::run()
{
    static SUBSCRIBE_VAR_t subscribe_var{0};

    // subscribe adc_data from adc hardware
    adc_report_s adc_data{0};
    uORB::Subscription adc_report_sub{ORB_ID(adc_report)};

    // subscribe pwm_data from pwm_input
    pwm_input_s pwm_input_data{0};
    uORB::Subscription pwm_input_sub{ORB_ID(pwm_input)};
    // subscribe pwm_data from pwm_capture
    pwm_captures_s pwm_captures_data_0{0};
    uORB::Subscription pwm_captures_sub_0{ORB_ID(pwm_captures_0)};
    pwm_captures_s pwm_captures_data_1{0};
    uORB::Subscription pwm_captures_sub_1{ORB_ID(pwm_captures_1)};

    pwm_captures_s pwm_captures_data_2{0};
    uORB::Subscription pwm_captures_sub_2{ORB_ID(pwm_captures_2)};
    pwm_captures_s pwm_captures_data_3{0};
    uORB::Subscription pwm_captures_sub_3{ORB_ID(pwm_captures_3)};

    pwm_encoder_me_s pwm_encoder_me_data{0};
    uORB::Subscription pwm_encoder_me_sub{ORB_ID(pwm_encoder_me)};

    // subscribe rc_data from remote controller
    rc_channels_s rc_data{0};
    uORB::Subscription rc_channels_sub{ORB_ID(rc_channels)};

    // subscribe motor_status from dshot thread
    orb_motor_status_me_s motor_status_data{0};
    uORB::Subscription orb_motor_status_me_sub{ORB_ID(orb_motor_status_me)};

    // subscribe debug_value
    debug_value_s debug_value_data{0};
    uORB::Subscription debug_value_sub{ORB_ID(debug_value)};

    // subscribe actuator_controls
    actuator_controls_s actuator_controls_data{0};
    uORB::Subscription actuator_controls_sub{ORB_ID(actuator_controls_0)};

    // subscribe vehicle_attitude_s
    vehicle_attitude_s vehicle_attitude_data{0};
    uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

    // subscribe sensor_mag
    sensor_mag_s sensor_mag_data{0};
    uORB::Subscription sensor_mag_sub{ORB_ID(sensor_mag)};

    // subscribe sensor_accel
    sensor_accel_s sensor_accel_data{0};
    uORB::Subscription sensor_accel_sub{ORB_ID(sensor_accel)};

    // subscribe key_command for mavros-mavlink test
    key_command_s key_command_data{0};
    uORB::Subscription key_command_sub{ORB_ID(key_command)};

    PX4_INFO("px4_cn is running");

    while(!should_exit())
    {
#ifdef PRINT_PWM_IN
        float counter_freq = 9e6;
        if(pwm_input_sub.update(&pwm_input_data))
        {
            static uint32_t pub_counter = 0;
            subscribe_var.pwm_pulse_width = pwm_input_data.pulse_width;
            subscribe_var.pwm_period = pwm_input_data.period;
//            float freq = counter_freq / ((float)subscribe_var.pwm_period);
//            float speed_rpm = freq * 60 / 12;
//            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
//            if(angle > 360)
//                angle = 360;
//            else if(angle < 0)
//                angle = 0;
//            angle -= ROTOR_ANGLE_BIAS;
//            if(angle < 0) angle += 360;
//            angle = 360 - angle;
//            PX4_INFO("width = %d, period = %d, angle = %.3f, freq = %.3f, speed = %.3f rpm",
//                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle, freq, speed_rpm);

            //begin encoder angle conversion
//            int32_t current_counter = int32_t(pwm_input_data.pulse_width);
//            static int32_t last_counter = 0;
//            int32_t delta_counter = 0;
//            static int64_t total_counter = 0;
//            double angle_total, angle_one_rot;
//
//            delta_counter = current_counter - last_counter;
//            if (delta_counter < -30000) delta_counter += 65536; //65535->0, up overflow
//            if (delta_counter > 30000) delta_counter -= 65536;  //0->65535, down overflow
//            total_counter += delta_counter;
//            angle_total = double(total_counter) / (2000.0f * 12.25f) * 360.0f;//encoder line(500) * 4 * gear ratio(21 or 12.25)
//            angle_one_rot = fmod(angle_total, 360.0f);
//            if (angle_one_rot < 0) angle_one_rot += 360.0f;
//            angle_one_rot = math::constrain((double)angle_one_rot, 0.0d, 360.0d);
//            last_counter = current_counter;
            //end encoder angle conversion
//            PX4_INFO("counter = %d, angle_total = %.3f, angle_one_rot = %.3f, pub_counter = %d",
//                     subscribe_var.pwm_pulse_width, angle_total, angle_one_rot, ++pub_counter);
        }

        if(pwm_encoder_me_sub.update(&pwm_encoder_me_data))
        {
            PX4_INFO("counter = %d, angle_total = %d, angle_total = %.3f, angle_one_rot = %.3f",
                     pwm_encoder_me_data.counter_reg, pwm_encoder_me_data.counter_total,
                     pwm_encoder_me_data.angle_total, pwm_encoder_me_data.angle_one_rot);
        }

        if(pwm_captures_sub_0.update(&pwm_captures_data_0))
        {
            subscribe_var.pwm_pulse_width = pwm_captures_data_0.pulse_width;
            subscribe_var.pwm_period = pwm_captures_data_0.period;
            float freq = counter_freq / ((float)subscribe_var.pwm_period);
            float speed_rpm = freq * 60 / 12;
            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
            if(angle > 360)
                angle = 360;
            else if(angle < 0)
                angle = 0;
            angle -= ROTOR_ANGLE_BIAS;
            if(angle < 0) angle += 360;
            angle = 360 - angle;
            PX4_INFO("G1, CAP1, width = %d, period = %d, angle = %.3f, freq = %.3f, speed = %.3f rpm",
                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle, freq, speed_rpm);
        }

        if(pwm_captures_sub_1.update(&pwm_captures_data_1))
        {
            subscribe_var.pwm_pulse_width = pwm_captures_data_1.pulse_width;
            subscribe_var.pwm_period = pwm_captures_data_1.period;
            float freq = counter_freq / ((float)subscribe_var.pwm_period);
            float speed_rpm = freq * 60 / 12;
            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
            if(angle > 360)
                angle = 360;
            else if(angle < 0)
                angle = 0;
            angle -= ROTOR_ANGLE_BIAS;
            if(angle < 0) angle += 360;
            angle = 360 - angle;
            PX4_INFO("G1, CAP3, width = %d, period = %d, angle = %.3f, freq = %.3f, speed = %.3f rpm",
                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle, freq, speed_rpm);
        }

        if(pwm_captures_sub_2.update(&pwm_captures_data_2))
        {
            subscribe_var.pwm_pulse_width = pwm_captures_data_2.pulse_width;
            subscribe_var.pwm_period = pwm_captures_data_2.period;
            float freq = counter_freq / ((float)subscribe_var.pwm_period);
            float speed_rpm = freq * 60 / 12;
            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
            if(angle > 360)
                angle = 360;
            else if(angle < 0)
                angle = 0;
            angle -= ROTOR_ANGLE_BIAS;
            if(angle < 0) angle += 360;
            angle = 360 - angle;
            PX4_INFO("G2, CAP1, width = %d, period = %d, angle = %.3f, freq = %.3f, speed = %.3f rpm",
                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle, freq, speed_rpm);
        }

        if(pwm_captures_sub_3.update(&pwm_captures_data_3))
        {
            subscribe_var.pwm_pulse_width = pwm_captures_data_3.pulse_width;
            subscribe_var.pwm_period = pwm_captures_data_3.period;
            float freq = counter_freq / ((float)subscribe_var.pwm_period);
            float speed_rpm = freq * 60 / 12;
            float angle = ((float)subscribe_var.pwm_pulse_width - 293) / (9698 - 293) * 360;
            if(angle > 360)
                angle = 360;
            else if(angle < 0)
                angle = 0;
            angle -= ROTOR_ANGLE_BIAS;
            if(angle < 0) angle += 360;
            angle = 360 - angle;
            PX4_INFO("G2, CAP2, width = %d, period = %d, angle = %.3f, freq = %.3f, speed = %.3f rpm",
                     subscribe_var.pwm_pulse_width, subscribe_var.pwm_period, angle, freq, speed_rpm);
        }
#endif

#ifdef PRINT_RC
        if(rc_channels_sub.update(&rc_data))
        {
            subscribe_var.rc_LX = (double)rc_data.channels[3];
            subscribe_var.rc_LY = (double)rc_data.channels[1];
            subscribe_var.rc_RX = (double)rc_data.channels[0];
            subscribe_var.rc_RY = (double)rc_data.channels[2];
//             PX4_INFO("rc = %f %f %f %f", subscribe_var.rc_LX, subscribe_var.rc_LY, subscribe_var.rc_RX, subscribe_var.rc_RY);
            PX4_INFO("0=%.3f, 1=%.3f, 2=%.3f, 3=%.3f, 4=%.3f, 5=%.3f, 6=%.3f,"
                     "7=%.3f, 8=%.3f, 9=%.3f, 10=%.3f, 11=%.3f, 12=%.3f, 13=%.3f",
                     rc_data.channels[0], rc_data.channels[1],
                     rc_data.channels[2], rc_data.channels[3],
                     rc_data.channels[4], rc_data.channels[5],
                     rc_data.channels[6], rc_data.channels[7],
                     rc_data.channels[8], rc_data.channels[9],
                     rc_data.channels[10], rc_data.channels[11],
                     rc_data.channels[12], rc_data.channels[13]
            );
        }
#endif

#ifdef PRINT_MOTOR_STATUS
        if(orb_motor_status_me_sub.update(&motor_status_data))
        {
            static uint32_t counter = 0;
            counter++;
            subscribe_var.motor_angle_deg = motor_status_data.angle_deg[4];
            subscribe_var.motor_speed_rpm = motor_status_data.speed_rpm[4];
            subscribe_var.dt_s = motor_status_data.dt[4];
            if(counter > 10)
            {
                PX4_INFO("deg = %.3f, rpm = %.3f, dt = %.3f, cnt = %d, out = %d, "
                         "sin = %.3f, amp = %.3f, pha = %.3f",
                         subscribe_var.motor_angle_deg, subscribe_var.motor_speed_rpm, subscribe_var.dt_s,
                         motor_status_data.frame_counter, motor_status_data.throttle_out[4],
                         motor_status_data.throttle_sin[4], motor_status_data.sin_amp[4],
                         motor_status_data.sin_pha[4]);
                counter = 0;
            }
        }
#endif

#ifdef PRINT_DEBUG_VALUE
        if(debug_value_sub.update(&debug_value_data))
        {
            static float last_value;
            PX4_INFO("debug: ind = %d, value = %.3f", debug_value_data.ind, debug_value_data.value - last_value);
            last_value = debug_value_data.value;
        }
#endif

#ifdef PRINT_ACTUATOR_CONTROL
        if(actuator_controls_sub.update(&actuator_controls_data))
        {
            PX4_INFO("actuator_controls = %.3f, %.3f, %.3f, %.3f",
                     actuator_controls_data.control[0],
                     actuator_controls_data.control[1],
                     actuator_controls_data.control[2],
                     actuator_controls_data.control[3]);
        }
#endif

#ifdef PRINT_VEHICLE_CONTROL
        if(vehicle_attitude_sub.update(&vehicle_attitude_data))
        {
            matrix::Quatf att_q(vehicle_attitude_data.q);
            // for stabilized attitude generation only extract the heading change from the delta quaternion
            const float rad2deg = 57.3;
            float roll_angle = matrix::Eulerf(att_q).phi() * rad2deg;
            float pitch_angle = matrix::Eulerf(att_q).theta() * rad2deg;
            float yaw_angle = matrix::Eulerf(att_q).psi() * rad2deg;
//            PX4_INFO("vehicle_attitude_data = %.3f, %.3f, %.3f, %.3f",
//                     vehicle_attitude_data.q[0],
//                     vehicle_attitude_data.q[1],
//                     vehicle_attitude_data.q[2],
//                     vehicle_attitude_data.q[3]);
            PX4_INFO("vehicle_attitude_data = %.3f, %.3f, %.3f",
                     roll_angle, pitch_angle, yaw_angle);
        }
#endif

#ifdef PRINT_SENSOR_MAG
        if(sensor_mag_sub.update(&sensor_mag_data))
        {
            PX4_INFO("sensor_mag_data = %.3f, %.3f, %.3f",
                     sensor_mag_data.x, sensor_mag_data.y, sensor_mag_data.z);
        }
#endif

#ifdef PRINT_SENSOR_ACCEL
        if(sensor_accel_sub.update(&sensor_accel_data))
        {
            PX4_INFO("sensor_accel_data = %.3f, %.3f, %.3f",
                     sensor_accel_data.x, sensor_accel_data.y, sensor_accel_data.z);
        }
#endif

#ifdef PRINT_KEY_COMMAND
        if(key_command_sub.update(&key_command_data))
        {
            PX4_INFO("key_command_sub = %c", key_command_data.cmd);
        }
#endif

//        PX4_INFO("PX4_INFO");
//        sleep(1);//parameter is int
        usleep(10000);//10ms
    }
}


void PX4cn::parameters_update(bool force)
{}


int PX4cn::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


