/**
 *  POWERFEATHER 4-CLAUSE LICENSE
 *
 *  Copyright (C) 2023, PowerFeather.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *      list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *
 *  3. Neither the name of PowerFeather nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 *  4. This software, with or without modification, must only be run on official
 *      PowerFeather boards.
 *
 *  THIS SOFTWARE IS PROVIDED BY POWERFEATHER “AS IS” AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL POWERFEATHER OR CONTRIBUTORS BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <esp_log.h>

#include "MasterI2C.h"

#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#ifndef ARDUINO

namespace PowerFeather
{
    static const char *TAG = "PowerFeather::Utils::MasterI2C";

    // I2C device addresses
    static constexpr uint8_t BQ2562X_I2C_ADDRESS = 0x6A;
    static constexpr uint8_t LC709204F_I2C_ADDRESS = 0x0B;

    bool MasterI2C::start()
    {
        // Configure I2C master bus
        i2c_master_bus_config_t bus_config = {};
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.i2c_port = static_cast<i2c_port_t>(_port);
        bus_config.scl_io_num = static_cast<gpio_num_t>(_sclPin);
        bus_config.sda_io_num = static_cast<gpio_num_t>(_sdaPin);
        bus_config.glitch_ignore_cnt = 7;
        bus_config.flags.enable_internal_pullup = false;

        ESP_LOGD(TAG, "Start with port: %d, sda: %d, scl: %d, freq: %d.", _port, _sdaPin, _sclPin, static_cast<int>(_freq));

        esp_err_t ret = i2c_new_master_bus(&bus_config, &_bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
            return false;
        }

        // Add charger device (BQ2562x)
        i2c_device_config_t charger_dev_cfg = {};
        charger_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        charger_dev_cfg.device_address = BQ2562X_I2C_ADDRESS;
        charger_dev_cfg.scl_speed_hz = _freq;

        ret = i2c_master_bus_add_device(_bus_handle, &charger_dev_cfg, &_dev_handle_charger);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add charger device: %s", esp_err_to_name(ret));
            i2c_del_master_bus(_bus_handle);
            return false;
        }

        // Add fuel gauge device (LC709204F)
        i2c_device_config_t fuel_gauge_dev_cfg = {};
        fuel_gauge_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        fuel_gauge_dev_cfg.device_address = LC709204F_I2C_ADDRESS;
        fuel_gauge_dev_cfg.scl_speed_hz = _freq;

        ret = i2c_master_bus_add_device(_bus_handle, &fuel_gauge_dev_cfg, &_dev_handle_fuel_gauge);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add fuel gauge device: %s", esp_err_to_name(ret));
            i2c_master_bus_rm_device(_dev_handle_charger);
            i2c_del_master_bus(_bus_handle);
            return false;
        }

        ESP_LOGD(TAG, "I2C master bus initialized successfully");
        return true;
    }

    bool MasterI2C::write(uint8_t address, uint8_t reg, const uint8_t *buf, size_t len)
    {
        uint8_t buf2[len + sizeof(reg)];
        memcpy(buf2, &reg, sizeof(reg));
        memcpy(&(buf2[sizeof(reg)]), buf, len);
        ESP_LOGV(TAG, "Write address: %02x, reg: %02x, buf: %p, len: %d.", address, reg, buf, len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);

        // Select the appropriate device handle
        i2c_master_dev_handle_t dev_handle = (address == BQ2562X_I2C_ADDRESS) ? _dev_handle_charger : _dev_handle_fuel_gauge;

        esp_err_t ret = i2c_master_transmit(dev_handle, buf2, sizeof(buf2), 1000 / portTICK_PERIOD_MS);
        return ret == ESP_OK;
    }

    bool MasterI2C::read(uint8_t address, uint8_t reg, uint8_t *buf, size_t len)
    {
        ESP_LOGV(TAG, "Read address: %02x, reg: %02x, buf: %p, len: %d.", address, reg, buf, len);

        // Select the appropriate device handle
        i2c_master_dev_handle_t dev_handle = (address == BQ2562X_I2C_ADDRESS) ? _dev_handle_charger : _dev_handle_fuel_gauge;

        esp_err_t res = i2c_master_transmit_receive(dev_handle, &reg, sizeof(reg), buf, len, 1000 / portTICK_PERIOD_MS);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);
        return res == ESP_OK;
    }

    bool MasterI2C::end()
    {
        ESP_LOGD(TAG, "End");

        esp_err_t ret1 = i2c_master_bus_rm_device(_dev_handle_charger);
        esp_err_t ret2 = i2c_master_bus_rm_device(_dev_handle_fuel_gauge);
        esp_err_t ret3 = i2c_del_master_bus(_bus_handle);

        return (ret1 == ESP_OK && ret2 == ESP_OK && ret3 == ESP_OK);
    }
}

#endif