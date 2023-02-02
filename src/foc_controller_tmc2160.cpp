#include <foc_controller_tmc2160.h>
#define EN_PIN 15  // Enable

FOCController::FOCController() {
    init_sine_quart();
}


FOCController::FOCController(AS5048A* m_encoder, TMC2160Stepper* driver,
    int16_t max_current_rms_nom_mA, float max_torque_nom, float foc_current_overdrive, SemaphoreHandle_t SPI_mutex):
    motor_encoder(m_encoder), driver(driver), max_current_rms_nom_mA(max_current_rms_nom_mA),
    max_torque_nom(max_torque_nom), foc_current_overdrive(foc_current_overdrive), foc_spi_mutex(SPI_mutex)
{

    motor_torque_const = max_torque_nom / (1.41 * max_current_rms_nom_mA);
    max_current_rms_foc_mA = max_current_rms_nom_mA * (1.0 + foc_current_overdrive);

    //generate foc_output_const: target-range: -255,...,+255; resolution (14 bit)
    this->foc_output_const = (255.0 / 8191.0) * (1.0 / (max_torque_nom * (1.0 + foc_current_overdrive)));

    this->foc_torque_command_spinlock = portMUX_INITIALIZER_UNLOCKED;

    init_sine_quart();
}

void FOCController::setup_driver() {

    Serial.println("DRVSYS_FOC_INFO: Initialize TMC2160 Stepper Driver");
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);

    driver->begin();           //  SPI: Init CS pins and possible SW SPI pins

    Serial.print("DRVSYS_FOC_INFO: DRV_STATUS=0b");
    Serial.println(driver->DRV_STATUS(), BIN);


    driver->toff(5);           // Enables driver in software
    driver->rms_current(max_current_rms_foc_mA); // Set motor RMS current
    driver->microsteps(256);   // Set microsteps to 1/16th

    driver->en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
    driver->pwm_autoscale(true); // Needed for stealthChop

    driver->direct_mode(true);

    xSemaphoreGive(foc_spi_mutex);

    init_sine_quart();

}
void FOCController::calibrate_motor_electric_angle() {


    if (read_calibration_data()) {
        return;
    }

    Serial.println("DRVSYS_FOC_INFO: Starting electric angle initial calibration.");
    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);

    const int n_steps = 200;

    driver->direct_mode(true);
    driver->rms_current(max_current_rms_foc_mA);
    //Reduce phase current to nominal value in stepper mode, to remain a cooler motor

    driver->coil_A(0);
    driver->coil_B(0);

    const int settle_time_ms = 50;
    const int n_samples = 10;
    const double step_angle = 1.8;
    int32_t lookup_offset = 0;

    int32_t table[200] = { 0 };
    int32_t delta_forw[200] = { 0 };
    int32_t delta_backw[200] = { 0 };
    int32_t angle_forw[200] = { 0 };
    int32_t angle_backw[200] = { 0 };

    int full_iterations = 5;

    for (int iter = 0; iter < full_iterations; iter++) {

        for (int i = 0; i < n_steps; i++) {

            int16_t ia = 255 * cos(50.0 * step_angle * DEG_TO_RAD * i);
            int16_t ib = 255 * sin(50.0 * step_angle * DEG_TO_RAD * i);

            driver->coil_A(ia);
            driver->coil_B(ib);
            delay(settle_time_ms);

            int32_t motor_angle = 0;
            for (int j = 0; j < n_samples; j++) {
                motor_angle += motor_encoder->getRotation(true);
            }
            motor_angle = motor_angle / n_samples;

            if (i == 0) {
                lookup_offset = motor_angle;
            }

            Serial.print("Motor Step: ");
            Serial.print(i);
            Serial.print(" Encoder Value: ");
            Serial.print(motor_angle);
            Serial.print(" Expected Angle: ");
            int32_t exp_angle = float(i) * (step_angle / 360) * 16383;
            int32_t delta = exp_angle - (motor_angle);
            Serial.print(exp_angle);
            Serial.print(" Delta: ");
            Serial.println(delta);

            delta_forw[i] += delta;
            angle_forw[i] += motor_angle - lookup_offset;
        }

        for (int i = n_steps - 1; i >= 0; i--) {
            if (i == n_steps - 1) {
                //
            }
            else {
                int16_t ia = 255 * cos(50 * 1.8 * DEG_TO_RAD * i);
                int16_t ib = 255 * sin(50 * 1.8 * DEG_TO_RAD * i);
                driver->coil_A(ia);
                driver->coil_B(ib);
                delay(settle_time_ms);
            }

            int32_t motor_angle = 0;
            for (int j = 0; j < n_samples; j++) {
                motor_angle += motor_encoder->getRotation(true);
            }
            motor_angle = motor_angle / n_samples;


            Serial.print("Motor Step: ");
            Serial.print(i);
            Serial.print(" Encoder Value: ");
            Serial.print(motor_angle);
            Serial.print(" Expected Angle: ");
            int32_t exp_angle = float(i) * (1.8 / 360) * 16383;
            int32_t delta = exp_angle - (motor_angle);
            Serial.print(exp_angle);
            Serial.print(" Delta: ");
            Serial.println(delta);


            delta_backw[i] += delta;
            angle_backw[i] += motor_angle - lookup_offset;
        }
    }

    for (int i = 0; i < n_steps; i++) {
        delta_backw[i] = delta_backw[i] / full_iterations;
        angle_backw[i] = angle_backw[i] / full_iterations;

        delta_forw[i] = delta_forw[i] / full_iterations;
        angle_forw[i] = angle_forw[i] / full_iterations;

    }

    // calculate offset angle
    int32_t offset_data = 0;

    for (int i = 0; i < n_steps; i++) {

        offset_data += (delta_backw[i] + delta_forw[i]) / 2;
        table[i] = (angle_backw[i] + angle_forw[i]) / 2;
    }

    offset_angle = offset_data / n_steps;
    Serial.print("Average Excenter Offset: ");
    Serial.print(offset_angle);
    Serial.print(" Lookup Offset: ");
    Serial.println(lookup_offset);

    for (int i = 0; i < n_steps; i++) {
        table[i] = table[i];
        Serial.println(table[i]);
    }

    Serial.print("Offset angle: ");
    Serial.println(offset_angle);


    for (int i = 0; i < 200; i++) {

        /*
        Serial.print("i ");
        Serial.println(i);
        */
        int x0 = 0;
        if (i == 0) {
            x0 = 0;
        }
        else {
            x0 = table[i];
        }

        int x1 = 0;
        if (i + 1 == 200) {
            x1 = 16383;

        }
        else {
            x1 = table[i + 1];
        }

        int y0 = float(i) * (1.8 / 360) * 16383;
        int y1 = float(i + 1) * (1.8 / 360) * 16383;
        /*
        Serial.print(y0);
        Serial.print(" - ");
        Serial.print(y1);

        Serial.print(" ---- ");
        Serial.print(x0);
        Serial.print(" - ");
        Serial.print(x1);
        Serial.println();
        */

        for (int j = x0; j <= x1; j++) {
            calibration_lookup[j] = y0 + (j - x0) * float(double(y1 - y0) / double(x1 - x0));;
        }
    }

    Serial.println("FOC Calibration Results: ");
    Serial.print("Offset Angle: ");
    Serial.println(offset_angle);
    Serial.print("Lookup Offset: ");
    Serial.println(lookup_offset);

    Serial.println("Calibration Lookup: ");
    Serial.print("{");
    for (int i = 0; i < n_steps; i++) {
        Serial.print(table[i]);
        if (i != n_steps - 1) {
            Serial.print(",");
        }
    }
    Serial.println("}");

    save_calibration_data(lookup_offset, offset_angle, table);
    /*
    for (int i = 0; i < n_steps; i++) {

        int16_t ia = 255 * cos(50.0 * step_angle * DEG_TO_RAD * i);
        int16_t ib = 255 * sin(50.0 * step_angle * DEG_TO_RAD * i);

        driver->coil_A(ia);
        driver->coil_B(ib);
        delay(settle_time_ms);

        int32_t motor_angle = 0;
        for (int j = 0; j < n_samples; j++) {
            motor_angle += motor_encoder->getRotation(true);
        }
        motor_angle = motor_angle / n_samples;

        /*
        if (i == 0) {
            lookup_offset = motor_angle;
        }
        */

        /*
     Serial.print("Motor Step: ");
     Serial.print(i);
     Serial.print(" Encoder Value: ");
     Serial.print(motor_angle);
     Serial.print(" Expected Angle: ");
     int32_t exp_angle = float(i) * (step_angle / 360) * 16383;
     Serial.print(exp_angle);
     Serial.print(" look up angle ");

     int encoder_input = motor_angle - lookup_offset;

     encoder_input = encoder_input % 16383;

     if (encoder_input < 0) {
         encoder_input = 16383 + encoder_input;
     }

     Serial.print(calibration_lookup[encoder_input]);

     Serial.print(" Align Output ");
     Serial.println(calibration_lookup[encoder_input] + lookup_offset + offset_angle);


 }
 */

    driver->rms_current(max_current_rms_foc_mA); // Set motor RMS current
    driver->direct_mode(true);
    xSemaphoreGive(foc_spi_mutex);

    Serial.println("DRVSYS_FOC_INFO: Succesfully calibrated Electric Angle");

}

void FOCController::set_foc_calibration(bool reset) {

    if (reset) {
        foc_pref.begin(cali_data_ns, false);

        foc_pref.putBool(drive_calibration_available, false);

        foc_pref.end();

        Serial.println("FOC: Reset Calibration. Recalibration required.");
    }

}

int32_t FOCController::get_calibrated_encoder_val(int32_t enc_val) {

    int encoder_input = enc_val - lookup_offset;

    encoder_input = encoder_input % 16383;

    if (encoder_input < 0) {
        encoder_input = 16383 + encoder_input;
    }

    return calibration_lookup[encoder_input] + lookup_offset + offset_angle;
}

bool FOCController::read_calibration_data() {

    foc_pref.begin(cali_data_ns, false);

    if (!foc_pref.getBool(drive_calibration_available)) {
        foc_pref.end();
        return false;
    }

    lookup_offset = foc_pref.getInt(lookup_offset_key, 0);
    offset_angle = foc_pref.getInt(offset_key, 0);

    const size_t lookup_size = 200;
    int32_t table[lookup_size] = { 0 };
    const size_t lookup_size_bytes = 4 * lookup_size;

    foc_pref.getBytes(lookup_table_key, table, lookup_size_bytes);

    foc_pref.end();


    // interpolate full lookup table
    Serial.println("Generate full interpolated calibration lookup table.");
    for (int i = 0; i < 200; i++) {

        int x0 = 0;
        if (i == 0) {
            x0 = 0;
        }
        else {
            x0 = table[i];
        }

        int x1 = 0;
        if (i + 1 == 200) {
            x1 = 16383;

        }
        else {
            x1 = table[i + 1];
        }

        int y0 = float(i) * (1.8 / 360) * 16383;
        int y1 = float(i + 1) * (1.8 / 360) * 16383;

        for (int j = x0; j <= x1; j++) {
            calibration_lookup[j] = y0 + (j - x0) * float(double(y1 - y0) / double(x1 - x0));;
        }

    }

    Serial.println("Succesfully loaded Phase Angle Calibration Data.");

    return true;


}
void FOCController::save_calibration_data(int32_t lookup_offset, int32_t offset_angle, int32_t* lookup) {

    foc_pref.begin(cali_data_ns, false);
    foc_pref.putInt(offset_key, offset_angle);
    foc_pref.putInt(lookup_offset_key, lookup_offset);

    const size_t lookup_size_bytes = 200 * 4;
    foc_pref.putBytes(lookup_table_key, lookup, lookup_size_bytes);
    foc_pref.putBool(drive_calibration_available, true);

    foc_pref.end();

    Serial.println("FOC: Saved Calibration Data on Flash");

}

void FOCController::set_target_torque(float torque_target) {

    float max_torque = max_torque_nom * (1.0 + foc_current_overdrive);
    if (torque_target > max_torque) {
        torque_target = max_torque;
    }
    else if (torque_target < -max_torque) {
        torque_target = -max_torque;
    }

    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = torque_target;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);

    if (target_torque == 0.0) {
        driver->freewheel(0b01);
    }
}

int32_t sign(int32_t value) {
    if (value >= 0) {
        return 1;
    }
    else {
        return -1;
    }
}

void FOCController::foc_control() {
    // variables for velocity compensation

    //static long last_micros = 0;

    //static float delay_normalizer_us = 1.0 / 100.0; // 1/200us

    // long current_micros = micros();

     //long passed_micros = current_micros - last_micros;


     //float  delta_angle_factor = passed_micros * delay_normalizer_us;

     //Serial.println(passed_micros);
     //last_micros = current_micros;

    static int32_t prev_delta_angle = 0;
    static int32_t prev_angle = 0;

    // obtain current motor angle

    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    //int32_t motor_angle = motor_encoder->getRotation(true);
    int32_t motor_angle = get_calibrated_encoder_val(motor_encoder->getRotation(false));
    xSemaphoreGive(foc_spi_mutex);

    motor_angle = motor_angle + 1 * offset_angle;

    int32_t delta_angle = motor_angle - prev_angle;
    int32_t delta_delta_angle = delta_angle - prev_delta_angle;
    prev_delta_angle = delta_angle;
    prev_angle = motor_angle;

    delta_angle = 1.0 * (delta_angle + 0.0 * delta_delta_angle);

    // Filter velocity compensation
    delta_angle = 0.1 * delta_angle + 0.9 * prev_delta_angle;
    prev_delta_angle = delta_angle;


    // anticipate phase shift based on previous shift 
    // empirically compensates latency
    //2.75
    int32_t empiric_phase_shift = 2.5 * delta_angle * 1;

    empiric_phase_shift = empiric_phase_shift;

    phase_null_angle = 0;

    int motor_angle_cut = motor_angle;

    int32_t electric_angle_int = (motor_angle_cut - int32_t(phase_null_angle) + empiric_phase_shift) * N_pole_pairs;

    // calculate desired phase currents

    int add_phase = 0;

    if (target_torque > 0.05) {
        add_phase = 128;
    }
    else if (target_torque < -0.05) {
        add_phase = -128;
    }

    int16_t i_a_simp = double(-foc_output_const * target_torque * sine_lookup(electric_angle_int + add_phase));
    int16_t i_b_simp = double(foc_output_const * target_torque * sine_lookup(4096 + electric_angle_int + add_phase));

    // create phase current register values
    xdirect_tmc direct;
    direct.values.coil_A = i_a_simp;
    direct.values.coil_B = i_b_simp;


    // write into current register of the driver
    xSemaphoreTake(foc_spi_mutex, portMAX_DELAY);
    driver->XDIRECT(direct.reg);
    xSemaphoreGive(foc_spi_mutex);

}

/*
void FOCController::set_target_torque_9bit(int16_t torque_target) {
    static const float inv_max_torque = 1.0 / max_torque;
    if (torque_target > 255) {
        torque_target = 255;
    }
    else if (torque_target < -255) {
        torque_target = -255;
    }
    portENTER_CRITICAL(&foc_torque_command_spinlock);
    this->target_torque = float(torque_target / 255.0) * inv_max_torque;
    portEXIT_CRITICAL(&foc_torque_command_spinlock);
}
*/

void FOCController::init_sine_quart() {

    const double sine_res = FOC_SINE_LOOKUP_RES;

    for (int i = 0; i < FOC_SINE_LOOKUP_SIZE; i++) {
        sineQuart_14bit[i] = sin(sine_res * (PI / 180.0) * double(i)) * 8192;
        //Serial.println(sineQuart[i]);
        //Serial.println(sineQuart_14bit[i]);
        //Serial.println(i);

    }
}

void FOCController::set_empiric_phase_shift_factor(float factor) {
    this->empiric_phase_shift_factor = factor;
}


int32_t FOCController::sine_lookup(int32_t val) {
    static const double divisor = FOC_SINE_LOOKUP_DIVISOR_disc;
    int32_t sign = 1;
    if (val < 0) {
        val = -val;
        sign = -1;
    }

    while (val > 16383) {
        val = val - 16384;
    }

    if (val <= 4096) {
        uint16_t index = round(val * divisor);
        return sineQuart_14bit[index] * sign;
    }
    else if (val > 4096 && val <= 8192) {
        uint16_t index = round((val - 4096) * divisor);
        return sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    else if (val > 8192 && val <= 12288) {
        uint16_t index = round((val - 8192) * divisor);
        return (-1) * sineQuart_14bit[index] * sign;
    }
    else if (val > 12288) {
        uint16_t index = round((val - 12288) * divisor);
        return -sineQuart_14bit[FOC_SINE_LOOKUP_SIZE - 1 - index] * sign;
    }
    return 0;

}