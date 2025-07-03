#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L4CX_ENABLED

#include "AP_RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

// register addresses taken from ST VL53L4CX driver
#define VL53LX_IDENTIFICATION__MODEL_ID                      0x010F
#define VL53LX_IDENTIFICATION__MODULE_TYPE                   0x0110
#define VL53LX_SYSTEM__MODE_START                            0x0087
#define VL53LX_SYSTEM__INTERRUPT_CLEAR                       0x0086
#define VL53LX_RESULT__RANGE_STATUS                          0x0089
#define VL53LX_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096
#define VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A                  0x0060
#define VL53LX_RANGE_CONFIG__VCSEL_PERIOD_B                  0x0063
#define VL53LX_ALGO__PART_TO_PART_RANGE_OFFSET_MM            0x001E
#define VL53LX_SYSTEM__THRESH_RATE_HIGH                      0x0050
#define VL53LX_SYSTEM__STREAM_COUNT_CTRL                     0x0084
#define VL53LX_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1        0x00D0
#define VL53LX_ANA_CONFIG__POWERDOWN_GO1                     0x00E0
#define VL53LX_GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT        0x00F0
#define VL53LX_MCU_TO_HOST_BANK__WR_ACCESS_EN                0x0100

#define EXPECTED_MODEL_ID    0xEB
#define EXPECTED_MODULE_TYPE 0xAA

struct CalibrationData {
    uint16_t offset_mm = 0;
};

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
    bool VL53_Calibrate();
    bool VL53_setCalibration();
    bool VL53_getCalibration();
    void start_continuous(void);
    void timer(void);

    void write_register(uint16_t reg, uint8_t value);
    bool read_register(uint16_t reg, uint8_t &value);
    bool read_register16(uint16_t reg, uint16_t &value);
    bool write_register16(uint16_t reg, uint16_t value);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    CalibrationData calibrationData {};
    uint32_t sum_mm = 0;
    uint16_t counter = 0;
};

#endif // AP_RANGEFINDER_VL53L4CX_ENABLED
