#include "AP_RangeFinder_VL53L4CX.h"

#if AP_RANGEFINDER_VL53L4CX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define REG_MODEL_ID        VL53LX_IDENTIFICATION__MODEL_ID
#define REG_MODULE_TYPE     VL53LX_IDENTIFICATION__MODULE_TYPE
#define REG_SYSTEM_MODE_START VL53LX_SYSTEM__MODE_START
#define REG_SYSTEM_INTERRUPT_CLEAR VL53LX_SYSTEM__INTERRUPT_CLEAR
#define REG_RESULT_RANGE_STATUS VL53LX_RESULT__RANGE_STATUS
#define REG_RESULT_DISTANCE_MM VL53LX_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0
#define REG_RANGE_CONFIG_VCSEL_PERIOD_A VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A
#define REG_RANGE_CONFIG_VCSEL_PERIOD_B VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B
#define REG_OFFSET_MM VL53LX_ALGO__PART_TO_PART_RANGE_OFFSET_MM
#define EXPECTED_MODEL_ID    0xEB
#define EXPECTED_MODULE_TYPE 0xAA


AP_RangeFinder_VL53L4CX::AP_RangeFinder_VL53L4CX(RangeFinder::RangeFinder_State &_state,
                                                 AP_RangeFinder_Params &_params,
                                                 AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev) :
    AP_RangeFinder_Backend(_state, _params),
    dev(std::move(_dev)) {}

AP_RangeFinder_Backend *AP_RangeFinder_VL53L4CX::detect(RangeFinder::RangeFinder_State &_state,
                                                        AP_RangeFinder_Params &_params,
                                                        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                        DistanceMode mode)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L4CX *sensor = NEW_NOTHROW AP_RangeFinder_VL53L4CX(_state, _params, std::move(dev));
    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();
    bool ok = sensor->check_id() && sensor->init(mode);
    sensor->dev->get_semaphore()->give();

    if (!ok) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_RangeFinder_VL53L4CX::check_id(void)
{
    uint8_t model = 0, type = 0;
    if (!read_register(REG_MODEL_ID, model) ||
        !read_register(REG_MODULE_TYPE, type)) {
        return false;
    }
    if (model != EXPECTED_MODEL_ID || type != EXPECTED_MODULE_TYPE) {
        return false;
    }
    printf("Detected VL53L4CX on bus 0x%x\n", unsigned(dev->get_bus_id()));
    return true;
}

bool AP_RangeFinder_VL53L4CX::init(DistanceMode mode)
{
    // basic mode selection. values based on ST's default presets
    uint8_t range_setting = 0x03; // long
    if (mode == DistanceMode::Short) {
        range_setting = 0x01;
    } else if (mode == DistanceMode::Medium) {
        range_setting = 0x02;
    }

    // stop any previous measurement so we can configure the device
    write_register(REG_SYSTEM_MODE_START, 0x00);

    // apply range configuration
    write_register(REG_RANGE_CONFIG_VCSEL_PERIOD_A, range_setting);
    write_register(REG_RANGE_CONFIG_VCSEL_PERIOD_B, range_setting);

    // basic tuning similar to ST example code
    write_register(VL53LX_SYSTEM__THRESH_RATE_HIGH, 0x02);
    write_register16(VL53LX_SYSTEM__STREAM_COUNT_CTRL, 16000);
    write_register(VL53LX_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1, 0x01);
    write_register(VL53LX_ANA_CONFIG__POWERDOWN_GO1, 0x01);
    write_register(VL53LX_GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT, 0x01);
    write_register16(VL53LX_MCU_TO_HOST_BANK__WR_ACCESS_EN, 50000);

    start_continuous();
    dev->register_periodic_callback(
        50000, FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L4CX::timer, void));
    return true;
}

bool AP_RangeFinder_VL53L4CX::VL53_Calibrate()
{
    // simple averaging based offset calibration
    uint32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 5; i++) {
        uint16_t dist = 0;
        if (get_reading(dist)) {
            sum += dist;
            count++;
        }
        hal.scheduler->delay(50);
    }

    if (count == 0) {
        return false;
    }

    calibrationData.offset_mm = sum / count;
    return true;
}

bool AP_RangeFinder_VL53L4CX::VL53_setCalibration()
{
    return write_register16(REG_OFFSET_MM, calibrationData.offset_mm);
}

bool AP_RangeFinder_VL53L4CX::VL53_getCalibration()
{
    uint16_t offset = 0;
    if (!read_register16(REG_OFFSET_MM, offset)) {
        return false;
    }
    calibrationData.offset_mm = offset;
    return true;
}

void AP_RangeFinder_VL53L4CX::start_continuous(void)
{
    write_register(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    write_register(REG_SYSTEM_MODE_START, 0x40);
}

bool AP_RangeFinder_VL53L4CX::get_reading(uint16_t &reading_mm)
{
    uint8_t status = 0;
    if (!read_register(REG_RESULT_RANGE_STATUS, status)) {
        return false;
    }
    if ((status & 0x1F) == 0) {
        return false;
    }

    if (!read_register16(REG_RESULT_DISTANCE_MM, reading_mm)) {
        return false;
    }

    // apply calibration offset
    if (reading_mm > calibrationData.offset_mm) {
        reading_mm -= calibrationData.offset_mm;
    } else {
        reading_mm = 0;
    }

    // clear interrupt for next measurement
    write_register(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    return true;
}

void AP_RangeFinder_VL53L4CX::timer(void)
{
    uint16_t range_mm;
    if (get_reading(range_mm) && range_mm <= 6000) {
        sum_mm += range_mm;
        counter++;
    }
}

void AP_RangeFinder_VL53L4CX::update(void)
{
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_VL53L4CX::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg>>8), uint8_t(reg), value };
    dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L4CX::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg>>8), uint8_t(reg) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L4CX::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg), uint8_t(value >> 8), uint8_t(value) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L4CX::read_register16(uint16_t reg, uint16_t &value)
{
    uint8_t b[2] = { uint8_t(reg>>8), uint8_t(reg) };
    uint16_t v = 0;
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

#endif // AP_RANGEFINDER_VL53L4CX_ENABLED
