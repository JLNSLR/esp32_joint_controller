#ifndef FOC_CONTROLLER_TMC2160_H
#define FOC_CONTROLLER_TMC2160_H

#include <FreeRTOS.h>
#include <TMCStepper.h>
#include <joint_control_global_def.h>
#include <AS5048A.h>
#include <Preferences.h>

#define FOC_SINE_LOOKUP_RES  0.0219726
#define FOC_SINE_LOOKUP_SIZE 4096
#define FOC_SINE_LOOKUP_DIVISOR 2.84444 // 1/2°
#define FOC_SINE_LOOKUP_DIVISOR_disc 1 // 1/2°

#define RAD2_14BITINT 2607.435

#define N_MOTOR_POLE_PAIRS 50 //Standard Nema17 Nema23 Stepper Motor

//#define FOC_EMPIRIC_PHASE_ANGLE_OFFSET 15710//15715 //

#define FOC_DEBUG




class FOCController
{
public:
    FOCController();
    FOCController(AS5048A* m_encoder, TMC2160Stepper* driver, int16_t max_current_rms_nom_mA,
        float max_torque_nom, float foc_current_overdrive, SemaphoreHandle_t SPI_mutex);

    void setup_driver();
    void calibrate_phase_angle(uint32_t phase_angle_null = 0); // blocking function should only be called during setup phase

    void calibrate_motor_electric_angle();

    void foc_control();

    AS5048A* motor_encoder;
    TMC2160Stepper* driver;

    // Variables that define the current targets based on desired torque
    int16_t max_current_rms_nom_mA;
    float max_torque_nom = 0;
    float foc_current_overdrive = 0.2;

    const int N_pole_pairs = N_MOTOR_POLE_PAIRS;
    float target_torque = 0;

    bool calibrated = false;
    int32_t phase_offset_correct = 0;


    void set_target_torque(float torque_target);
    //void set_target_torque_9bit(int16_t torque_target);
    void set_empiric_phase_shift_factor(float factor);

    void _test_sineLookup(float input);

    void set_foc_calibration(bool reset);

    long microseconds = 0;
    bool  input_averaging = true;
    int32_t phase_null_angle = 0;
    float empiric_phase_shift_factor = 0;
    int16_t max_current_rms_foc_mA;
    float motor_torque_const;

private:
    union xdirect_tmc
    {
        uint32_t reg : 25;
        struct
        {
            int16_t coil_A : 9;
            int8_t : 7;
            int16_t coil_B : 9;
        } values;
    }direct;

    portMUX_TYPE foc_torque_command_spinlock;
    SemaphoreHandle_t foc_spi_mutex;

    void init_sine_quart();
    int32_t sine_lookup(int32_t val);

    int32_t predictive_angle_shift = 1.0;

    int32_t sineQuart_14bit[FOC_SINE_LOOKUP_SIZE] = { 0 };

    double foc_output_const;

    int32_t calibration_lookup[16384] = { 0 };
    int32_t lookup_offset = 0;
    int32_t offset_angle = 0;

    int32_t get_calibrated_encoder_val(int32_t enc_val);


    Preferences foc_pref;
    const char* drive_calibration_available = "cali";
    const char* lookup_offset_key = "loff";
    const char* offset_key = "offs";
    const char* lookup_table_key = "lt";

    const char* cali_data_ns = "foc";

    bool read_calibration_data();
    void save_calibration_data(int32_t lookup_offset, int32_t offset_angle, int32_t* lookup);





};

#endif // ! FOC_CONTROLLER_H
