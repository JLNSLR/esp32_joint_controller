#ifndef ROBOT_GLOBAL_DEF_H
#define ROBOT_GLOBAL_DEF_H

// Hybrid Stepper Motor Definitions

// All definitions made in IRMS -> Imax = sqrt(2) * Irms

#define JOINT_0_MOTOR_NOMINAL_CURRENT_RMS 2.8
#define JOINT_1_MOTOR_NOMINAL_CURRENT_RMS 2.8
#define JOINT_2_MOTOR_NOMINAL_CURRENT_RMS 2.1
#define JOINT_3_MOTOR_NOMINAL_CURRENT_RMS 2.1
#define JOINT_4_MOTOR_NOMINAL_CURRENT_RMS 1.68
#define JOINT_5_MOTOR_NOMINAL_CURRENT_RMS 1.5 
#define JOINT_6_MOTOR_NOMINAL_CURRENT_RMS 0.4

#define SQRT2 1.41421
// Kt_rms = 1.41*Kt

#define JOINT_0_MOTOR_HOLD_TORQUE 1.9
#define JOINT_1_MOTOR_HOLD_TORQUE 1.9
#define JOINT_2_MOTOR_HOLD_TORQUE 0.65
#define JOINT_3_MOTOR_HOLD_TORQUE 0.65
#define JOINT_4_MOTOR_HOLD_TORQUE 0.51
#define JOINT_5_MOTOR_HOLD_TORQUE 0.45
#define JOINT_6_MOTOR_HOLD_TORQUE 0.26

#define JOINT_0_TRANSMISSION_RATIO 60
#define JOINT_1_TRANSMISSION_RATIO 60
#define JOINT_2_TRANSMISSION_RATIO 50
#define JOINT_3_TRANSMISSION_RATIO 50
#define JOINT_4_TRANSMISSION_RATIO 43.94740484
#define JOINT_5_TRANSMISSION_RATIO 22.5
#define JOINT_6_TRANSMISSION_RATIO 6

// 0 -> base joint -> 6 -> endeffector joint
const float motor_nominal_current_rms[7] = { JOINT_0_MOTOR_NOMINAL_CURRENT_RMS, JOINT_1_MOTOR_NOMINAL_CURRENT_RMS,
                                                 JOINT_2_MOTOR_NOMINAL_CURRENT_RMS, JOINT_3_MOTOR_NOMINAL_CURRENT_RMS,
                                                 JOINT_4_MOTOR_NOMINAL_CURRENT_RMS, JOINT_5_MOTOR_NOMINAL_CURRENT_RMS , JOINT_6_MOTOR_NOMINAL_CURRENT_RMS };
const float motor_hold_torque[7] = { JOINT_0_MOTOR_HOLD_TORQUE,JOINT_1_MOTOR_HOLD_TORQUE,
                                    JOINT_2_MOTOR_HOLD_TORQUE,JOINT_3_MOTOR_HOLD_TORQUE,JOINT_4_MOTOR_HOLD_TORQUE
                                    ,JOINT_5_MOTOR_HOLD_TORQUE,JOINT_6_MOTOR_HOLD_TORQUE };
const float max_overdrive = 0.75;

const float joint_transmission_ratio[7] = { JOINT_0_TRANSMISSION_RATIO,JOINT_1_TRANSMISSION_RATIO, JOINT_2_TRANSMISSION_RATIO,
                                                JOINT_3_TRANSMISSION_RATIO, JOINT_4_TRANSMISSION_RATIO, JOINT_5_TRANSMISSION_RATIO,JOINT_6_TRANSMISSION_RATIO };

// DMS Based Torque Sensor definitions
#define JOINT_0_TORQUE_SENSOR_MAX_Nm 200.0
#define JOINT_1_TORQUE_SENSOR_MAX_Nm 200.0
#define JOINT_2_TORQUE_SENSOR_MAX_Nm 100.0
#define JOINT_3_TORQUE_SENSOR_MAX_Nm 100.0
#define JOINT_4_TORQUE_SENSOR_MAX_Nm 100.0
#define JOINT_5_TORQUE_SENSOR_MAX_Nm 50.0
#define JOINT_6_TORQUE_SENSOR_MAX_Nm 50.0

#define TORQUE_SENSOR_MAX_VAL23_BIT 8388608

#define UNSIGNED_INT_16BIT_VAL 65536
#define SIGNED_INT_16BIT_MAXVAL 32768

#define SIGNED_INT_14BIT_MAXVAL 8192
#define SIGNED_INT_13BIT_MAXVAL 4096
#define SIGNED_INT_12BIT_MAXVAL 2048
#define UNSIGNED_INT_12BIT_MAXVAL 4096
#define SIGNED_INT_20BIT_MAXVAL 524288

#define DEG2RAD 0.01745329251994329576923690768489
#define RAD2DEG 57.295779513082320876798154814105

#define MAX_VEL_DATA_DEG_PER_S 400 // (°/s)
#define MAX_POS_DATA_DEG 180.0 // °
#define MAX_ACC_DATA_DEG 1250.0 // °/s^2

#define MAX_MOTOR_TEMP 150.0

#define MAX_GAIN_VAL 1000.0

#define N_BITS_POS_DATA 14
#define N_BITS_VEL_DATA 14
#define N_BITS_ACC_DATA 13

#define N_BITS_POS_TARGET_DATA 16
#define N_BITS_VEL_TARGET_DATA 14
#define N_BITS_ACC_TARGET_DATA 13


#define N_BITS_TORQUE_SENSOR_DATA 24

const float motor_command_res = 1.0 / 255.0;

const float torque_communication_res = 1.0 / 1024.0;//11 Bit signed


const float motor_phase_current_rms_nom[7] = { JOINT_0_MOTOR_NOMINAL_CURRENT_RMS, JOINT_1_MOTOR_NOMINAL_CURRENT_RMS,
                                                    JOINT_2_MOTOR_NOMINAL_CURRENT_RMS, JOINT_3_MOTOR_NOMINAL_CURRENT_RMS,
                                                    JOINT_4_MOTOR_NOMINAL_CURRENT_RMS, JOINT_5_MOTOR_NOMINAL_CURRENT_RMS,
                                                    JOINT_6_MOTOR_NOMINAL_CURRENT_RMS };


const float motor_torque_const[7] = { JOINT_0_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_0_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_1_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_1_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_2_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_2_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_4_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_4_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_5_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_5_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_6_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_6_MOTOR_NOMINAL_CURRENT_RMS),
                                        JOINT_5_MOTOR_HOLD_TORQUE / (SQRT2 * JOINT_5_MOTOR_NOMINAL_CURRENT_RMS) };

const float motor_torque_conversion_factor_arr_9bit[7] = { float(JOINT_0_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[0] * motor_command_res,
                                                 float(JOINT_1_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[1] * motor_command_res,
                                                 float(JOINT_2_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[2] * motor_command_res,
                                                 float(JOINT_3_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[3] * motor_command_res,
                                                 float(JOINT_4_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[4] * motor_command_res,
                                                 float(JOINT_5_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[5] * motor_command_res,
                                                 float(JOINT_6_MOTOR_NOMINAL_CURRENT_RMS * SQRT2) * motor_torque_const[6] * motor_command_res };


const float motor_torque_conversion_factor_arr_11bit[7] = { motor_hold_torque[0] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[1] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[2] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[3] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[4] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[5] * float(1.0 + max_overdrive) * torque_communication_res,
                                                 motor_hold_torque[6] * float(1.0 + max_overdrive) * torque_communication_res };

const float sensor_torque_conversion_factor_24bit[7] = { JOINT_0_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_1_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_2_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_3_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_4_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_5_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
                                                            JOINT_6_TORQUE_SENSOR_MAX_Nm / float(TORQUE_SENSOR_MAX_VAL23_BIT),
};

const float pos_data_14bit_to_val = MAX_POS_DATA_DEG * DEG2RAD * (1.0 / SIGNED_INT_14BIT_MAXVAL);
const float vel_data_14bit_to_val = MAX_VEL_DATA_DEG_PER_S * DEG2RAD * (1.0 / SIGNED_INT_14BIT_MAXVAL);
const float acc_data_13bit_to_val = MAX_ACC_DATA_DEG * DEG2RAD * (1.0 / SIGNED_INT_13BIT_MAXVAL);

const float pos_data_16bit_to_val = MAX_POS_DATA_DEG * DEG2RAD * (1.0 / SIGNED_INT_16BIT_MAXVAL);



const float pos_to_16bit_data = 1.0 / pos_data_16bit_to_val;

const float pos_to_14bit_data = 1.0 / pos_data_14bit_to_val;
const float vel_to_14bit_data = 1.0 / vel_data_14bit_to_val;
const float acc_to_13bit_data = 1.0 / acc_data_13bit_to_val;

//const float ref_torque_data_14bit_to_val = float(JOINT_TORQUE_REF_MAX_Nm) / float(SIGNED_INT_14BIT_MAXVAL);
//const float ref_torque_to_14bit_data = 1.0 / ref_torque_data_14bit_to_val;

const float motor_temp_12bit_to_val = MAX_MOTOR_TEMP / UNSIGNED_INT_12BIT_MAXVAL;
const float motor_temp_val_to_12bit = 1.0 / motor_temp_12bit_to_val;

//const float pid_gain_data_20bit_to_val = MAX_GAIN_VAL * (1.0 / 524288);
//const float pid_gain_val_to_20bit = 1.0 / pid_gain_data_20bit_to_val;

const float gain_data_16bit_to_val = MAX_GAIN_VAL * (1.0 / UNSIGNED_INT_16BIT_VAL);
const float gain_val_to_16bit = 1.0 / gain_data_16bit_to_val;




#endif // !ROBOT_GLOBAL_DEF