#pragma once

#include <gpiod.h>
#include <string>

class GpioOutput
{
public:
    GpioOutput(const std::string &chip_name,
                unsigned int line_offset,
                const std::string &consumer = "warehouse_gcs");

    ~GpioOutput();

    GpioOutput(const GpioOutput &) = delete;
    GpioOutput &operator=(const GpioOutput &) = delete;

    void setHigh();
    void setLow();
    void write(int value);

private:
    gpiod_chip *chip_ = nullptr;
    gpiod_line *line_ = nullptr;
};