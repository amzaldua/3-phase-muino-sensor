#include "esphome.h"
#include "sensor_defs.h"
#include "muino_3phase_i2c.h"

namespace esphome {
namespace muino_3phase_i2c {

bool Muino3PhaseI2CSensor::phase_coarse(int a, int b, int c) {
    static bool first = true;

    int max = 1500;
    int min = 5;

    if (a < min || b < min || c < min ){
        ESP_LOGW("light_level", "Too dark, ignoring this measurement");
        return false;
    }

    if (a > max || b > max || c > max){
        ESP_LOGW("light_level", "Too bright, ignoring this measurement");
        return false;
    }

    if (!init_ok_) {
        state.liters = 0;
        state.phase = 0;
        state.a_min = a;
        state.b_min = b;
        state.c_min = c;
        state.a_max = 0;
        state.b_max = 0;
        state.c_max = 0;
        init_ok_ = true;
        return 0;
    }

    float alpha_cor = 0.001;
    if (state.liters < 2) {
        alpha_cor = 0.1; // when 2 liter not found correct harder
        if (state.liters < 0) {
            state.liters = 0;
        }
    }

    state.a_min = mini_average(state.a_min, a, alpha_cor);
    state.b_min = mini_average(state.b_min, b, alpha_cor);
    state.c_min = mini_average(state.c_min, c, alpha_cor);
    state.a_max = max_average(state.a_max, a, alpha_cor);
    state.b_max = max_average(state.b_max, b, alpha_cor);
    state.c_max = max_average(state.c_max, c, alpha_cor);

    short pn[5];
    if (state.phase & 1)
        pn[0] = a + a - b - c, pn[1] = b + b - a - c,
        pn[2] = c + c - a - b; // same
    else
        pn[0]     = b + c - a - a, // less
            pn[1] = a + c - b - b, // more
            pn[2] = a + b - c - c; // same
    pn[3] = pn[0], pn[4] = pn[1];

    short i = state.phase > 2 ? state.phase - 3 : state.phase;
    if (pn[i + 2] < pn[i + 1] && pn[i + 2] < pn[i]){
        time_since_last_flow_ = millis();

        if (flow_rate_sensor_ != nullptr) {
            flow_rate_increment_();
        }

        if (pn[i + 1] > pn[i])
            state.phase++;
        else
            state.phase--;
    }

    if (state.phase == 6) {
        state.liters++;
        state.phase = 0;

        update_consumption(1);
    }
    else if (state.phase == -1) {
        state.liters--;
        state.phase = 5;

        // Too many problem with that: It is useful?
        // update_consumption(-1);
    }

    return true;
}

void Muino3PhaseI2CSensor::update_consumption(int8_t value) {
    main_consumption += value;
    secondary_consumption += value;
    tertiary_consumption += value;

    current_consumption_ += value;

    if (value > 0) {
        index_ += 1;
    }
}

float Muino3PhaseI2CSensor::mini_average(float x, float y, float alpha_cor) {
    if ((x + 5) <= y && y > 10) {
        return x;
    } else {
        return (1 - alpha_cor) * x + alpha_cor * y;
    }
}

float Muino3PhaseI2CSensor::max_average(float x, float y, float alpha_cor) {
    if ((x - 5) >= y && y < 2500) {
        return x;
    } else {
        return (1 - alpha_cor) * x + alpha_cor * y;
    }
}

bool Muino3PhaseI2CSensor::write_(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    if (!this->write_bytes(addr, data, 2)) {
        ESP_LOGE(TAG, "Failed to write to 0x%02X", addr);
        return false;
    }
    return true;
}

bool Muino3PhaseI2CSensor::write_(uint8_t addr, uint8_t reg, uint8_t* val, size_t length) {
    uint8_t buffer[length + 1];
    buffer[0] = reg;
    memcpy(buffer + 1, val, length);
    if (!this->write_bytes(addr, buffer, length + 1)) {
        ESP_LOGE(TAG, "Failed to write to 0x%02X", addr);
        return false;
    }
    return true;
}

bool Muino3PhaseI2CSensor::read_(uint8_t addr, uint8_t reg, uint8_t* data, size_t length) {
    if (!this->write_bytes(addr, &reg, 1)) {
        ESP_LOGE(TAG, "Failed to write register to 0x%02X", addr);
        return false;
    }
    if (!this->read_bytes(addr, data, length)) {
        ESP_LOGE(TAG, "Failed to read from 0x%02X", addr);
        return false;
    }
    return true;
}

void Muino3PhaseI2CSensor::set_pin_io_(uint8_t pin_number, bool value) {
    uint8_t current;
    read_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, &current, 1);
    if (value)
        current |= (1 << pin_number);
    else
        current &= ~(1 << pin_number);

    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, current);
}

void Muino3PhaseI2CSensor::init_light_sensor_(uint8_t sensor_id) {
    uint8_t reg = 0x00;
    if (sensor_id == 0) {
        set_pin_io_(3, true);
    } else if (sensor_id == 1) {
        set_pin_io_(4, true);
    } else if (sensor_id == 2) {
        set_pin_io_(5, true);
    } else {
        return;
    }

    // Gain: x2, It: 100ms
    uint8_t data_wr[2] = {0x00, (1 << 0)};  
    write_(VEML6030_I2C_ADDR_H, VEML6030_ALS_SD, (uint8_t*)&data_wr, 2);

    if (sensor_id == 0) {
        set_pin_io_(3, false);
    } else if (sensor_id == 1) {
        set_pin_io_(4, false);
    } else if (sensor_id == 2) {
        set_pin_io_(5, false);
    } else {
        return;
    }
}

uint16_t Muino3PhaseI2CSensor::read_sensor_(uint8_t sensor_id) {
    uint16_t val = 0;

    if (sensor_id == 0)
        this->set_pin_io_(3, true);
    else if (sensor_id == 1)
        this->set_pin_io_(4, true);
    else if (sensor_id == 2)
        this->set_pin_io_(5, true);

    read_(VEML6030_I2C_ADDR_H, VEML6030_ALS_READ_REG, (uint8_t*)&val, 2);

    if (sensor_id == 0)
        set_pin_io_(3, false);
    else if (sensor_id == 1)
        set_pin_io_(4, false);
    else if (sensor_id == 2)
        set_pin_io_(5, false);

    return val;
}

void Muino3PhaseI2CSensor::flow_rate_push_value_(float value) {
    flow_rate_values_[flow_rate_index_] = value;
    flow_rate_index_ = (flow_rate_index_ + 1) % 5;

    float sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += flow_rate_values_[i];
    }
    flow_rate = sum / 5;
}

void Muino3PhaseI2CSensor::flow_rate_increment_() {
    constexpr float CONVERSION_FACTOR = 10.0; // 60 sec / 6 phases = 10

    float current_time = millis() / 1000.0;
    float delta_t = current_time - flow_rate_last_time_;

    flow_rate_push_value_(CONVERSION_FACTOR / delta_t);

    flow_rate_last_time_ = current_time;
}

void Muino3PhaseI2CSensor::flow_rate_reset_() {
    float current_time = millis() / 1000.0;
    float delta_t = current_time - flow_rate_last_time_;

    if (flow_rate_last_time_ > 0 && delta_t > flow_rate_reset_time_) {
        flow_rate_push_value_(0);
    }
}

void Muino3PhaseI2CSensor::save_consumptions(bool shutdown_occured) {
    static uint32_t index_saved = -1;
    static uint32_t main_saved = -1;
    static uint32_t secondary_saved = -1;
    static uint32_t tertiary_saved = -1;

    ESP_LOGI(TAG, "Saving consumptions");

    if (index_!= index_saved) {
        index_pref_.save(&index_);
        index_saved = index_;
    }

    if (main_consumption != main_saved) {
        main_pref_.save(&main_consumption);
        main_saved = main_consumption;
    }

    if (secondary_consumption != secondary_saved) {
        secondary_pref_.save(&secondary_consumption);
        secondary_saved = secondary_consumption;
    }

    if (tertiary_consumption != tertiary_saved) {
        tertiary_pref_.save(&tertiary_consumption);
        tertiary_saved = tertiary_consumption;
    }

    if (shutdown_occured && measurements_consistency_sensor_) {
        shutdown_consistency_pref_.save(&shutdown_value_);
    }
}

void Muino3PhaseI2CSensor::restore_consumptions(bool startup_occured) {
    if (!index_pref_.load(&index_)) {
        index_ = 0;
    }

    if (!main_pref_.load(&main_consumption)) {
        main_consumption = 0;
    }

    if (!secondary_pref_.load(&secondary_consumption)) {
        secondary_consumption = 0;
    }

    if (!tertiary_pref_.load(&tertiary_consumption)) {
        tertiary_consumption = 0;
    }

    if (startup_occured && measurements_consistency_sensor_) {
        uint8_t value = 0;
        shutdown_consistency_pref_.load(&value);
        measurements_consistency_sensor_->publish_state((value != shutdown_value_));

        value = 0;
        shutdown_consistency_pref_.save(&value);
    }

    update_values_();
}

void Muino3PhaseI2CSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Muino 3-Phase Sensor...");

    uint8_t data_wr;

    data_wr = SENS0 | SENS1 | SENS2 | LED;
    write_(PI4IO_I2C_ADDR, PI4IO_IO_DIRECTION, data_wr);

    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT, 0);

    data_wr = ~(SENS0 | SENS1 | SENS2 | LED);
    write_(PI4IO_I2C_ADDR, PI4IO_OUTPUT_HI_IMPEDANCE, data_wr);

    init_light_sensor_(0);
    init_light_sensor_(1);
    init_light_sensor_(2);

    state.liters = 0;
    state.phase = 0;
    state.a_min = 0;
    state.b_min = 0;
    state.c_min = 0;
    state.a_max = 0;
    state.b_max = 0;
    state.c_max = 0;

    previous_consumption_ = 0;
    current_consumption_ = 0;
    time_since_last_flow_ = 0;

    main_consumption = 0;
    secondary_consumption = 0;
    tertiary_consumption = 0;

    main_ml_reset = 0;
    secondary_ml_reset = 0;
    tertiary_ml_reset = 0;

    index_pref_ = global_preferences->make_preference<uint32_t>(1004);
    main_pref_ = global_preferences->make_preference<uint32_t>(1001);
    secondary_pref_ = global_preferences->make_preference<uint32_t>(1002);
    tertiary_pref_ = global_preferences->make_preference<uint32_t>(1003);

    flow_rate = 0;

    restore_consumptions(true);

    shutdown_consistency_pref_ = global_preferences->make_preference<uint8_t>(1020);
}

void Muino3PhaseI2CSensor::update_values_() {
    float ml_part = state.phase / 6.0;

    if (index_sensor_ != nullptr)
        index_sensor_->publish_state((uint32_t)index_);

    if (main_consumption_sensor_ != nullptr)
        main_consumption_sensor_->publish_state(
            (float)(main_consumption + (ml_part - main_ml_reset))
        );

    if (secondary_consumption_sensor_ != nullptr)
        secondary_consumption_sensor_->publish_state(
            (float)(secondary_consumption + (ml_part - secondary_ml_reset))
        );

    if (tertiary_consumption_sensor_ != nullptr)
        tertiary_consumption_sensor_->publish_state(
            (float)(tertiary_consumption) + (ml_part - tertiary_ml_reset)
        );

    if (time_since_last_flow_sensor_ != nullptr && time_since_last_flow_ != 0)
        time_since_last_flow_sensor_->publish_state((millis() - time_since_last_flow_) / 1000);

    if (previous_consumption_sensor_ != nullptr)
        previous_consumption_sensor_->publish_state(previous_consumption_);

    if (current_consumption_sensor_ != nullptr)
        current_consumption_sensor_->publish_state(current_consumption_);

    if (flow_rate_sensor_ != nullptr)
        flow_rate_sensor_->publish_state((float)flow_rate);
}

void Muino3PhaseI2CSensor::set_index(int value) {
    index_ = value;

    if (measurements_consistency_sensor_)
        measurements_consistency_sensor_->publish_state(false);
}

void Muino3PhaseI2CSensor::reset_total() {
    init_ok_ = false;
    state.liters = 0;
    state.phase = 0;
    previous_consumption_ = 0;
    current_consumption_ = 0;
    time_since_last_flow_ = 0;
    flow_rate = 0;
    update_values_();
    ESP_LOGI(TAG, "Total consumption reset to 0");
}

void Muino3PhaseI2CSensor::reset_main_consumption() {
    main_consumption = 0;
    main_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::reset_secondary_consumption() {
    secondary_consumption = 0;
    secondary_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::reset_tertiary_consumption() {
    tertiary_consumption = 0;
    tertiary_ml_reset = state.phase / 6.0;
    update_values_();
};

void Muino3PhaseI2CSensor::set_led(bool state) {
    this->set_pin_io_(6, state);
}

void Muino3PhaseI2CSensor::update() {
    static uint8_t status = 0;
    uint32_t now = millis();
    char buffer[50];

    if (millis() - last_update_ >= 100) {
        if (debug_mode_) {
            if (a_sensor_ != nullptr) {
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_a_dark_, sen_a_light_, state.a, history_a_[history_index_]
                );
                a_sensor_->publish_state(buffer);
            }

            if (b_sensor_ != nullptr) {
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_b_dark_, sen_b_light_, state.b, history_b_[history_index_]
                );
                b_sensor_->publish_state(buffer);
            }

            if (c_sensor_ != nullptr) {
                snprintf(
                    buffer, sizeof(buffer),
                    "[%d, %d] raw: %d, avg: %d",
                    sen_c_dark_, sen_c_light_, state.c, history_c_[history_index_]
                );
                c_sensor_->publish_state(buffer);
            }

            if (phase_sensor_ != nullptr)
                phase_sensor_->publish_state(state.phase);
        }

        update_values_();
        last_update_ = millis();
    }

    if (now - time_since_last_flow_ > 60000 && current_consumption_ > 0) {
        previous_consumption_ = current_consumption_;
        current_consumption_ = 0;
    }

    switch (status) {
    case 0:
        setup();
        update_values_();
        status = 1;
        break;
    case 1:
        sen_a_dark_ = read_sensor_(0);
        sen_b_dark_ = read_sensor_(1);
        sen_c_dark_ = read_sensor_(2);
        set_led(true);
        status = 2;
        break;
    case 2:
        sen_a_light_ = read_sensor_(0);
        sen_b_light_ = read_sensor_(1);
        sen_c_light_ = read_sensor_(2);
        set_led(false);
        history_a_[history_index_] = sen_a_light_ - sen_a_dark_;
        history_b_[history_index_] = sen_b_light_ - sen_b_dark_;
        history_c_[history_index_] = sen_c_light_ - sen_c_dark_;
        history_index_ = (history_index_ + 1) % 3;
        state.a = (history_a_[0] + history_a_[1] + history_a_[2]) / 3;
        state.b = (history_b_[0] + history_b_[1] + history_b_[2]) / 3;
        state.c = (history_c_[0] + history_c_[1] + history_c_[2]) / 3;
        break;
    default:
        status = 0;
        break;
    }
}

}  // namespace muino_3phase_i
