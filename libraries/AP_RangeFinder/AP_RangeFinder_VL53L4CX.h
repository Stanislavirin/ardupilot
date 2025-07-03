#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L4CX_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_VL53L4CX : public AP_RangeFinder_Backend
{
public:
    enum class DistanceMode { Short, Medium, Long };

    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                          DistanceMode mode);

    void update(void) override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override { return MAV_DISTANCE_SENSOR_LASER; }

private:
    AP_RangeFinder_VL53L4CX(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init(DistanceMode mode);
    bool check_id(void);
    bool get_reading(uint16_t &reading_mm);
    void start_continuous(void);
    void timer(void);

    void write_register(uint16_t reg, uint8_t value);
    bool read_register(uint16_t reg, uint8_t &value);
    bool read_register16(uint16_t reg, uint16_t &value);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    uint32_t sum_mm = 0;
    uint16_t counter = 0;
};

#endif // AP_RANGEFINDER_VL53L4CX_ENABLED
