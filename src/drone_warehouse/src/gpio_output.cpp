#include "drone_warehouse/gpio_output.hpp"

#include <stdexcept>

GpioOutput::GpioOutput(const std::string &chip_name,
                        unsigned int line_offset,
                        const std::string &consumer)
{
    chip_ = gpiod_chip_open_by_name(chip_name.c_str());
    if (!chip_) {
        throw std::runtime_error("Failed to open gpio chip: " + chip_name);
    }

    line_ = gpiod_chip_get_line(chip_, line_offset);
    if (!line_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        throw std::runtime_error("Failed to get gpio line: " + std::to_string(line_offset));
    }

    const int ret = gpiod_line_request_output(line_, consumer.c_str(), 0);
    if (ret < 0) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        line_ = nullptr;
        throw std::runtime_error("Failed to request gpio line as output");
    }
}

GpioOutput::~GpioOutput()
{
    if (line_) {
        gpiod_line_release(line_);
        line_ = nullptr;
    }

    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

void GpioOutput::setHigh()
{
    write(1);
}

void GpioOutput::setLow()
{
    write(0);
}

void GpioOutput::write(int value)
{
    if (!line_) {
        throw std::runtime_error("GPIO line is not initialized");
    }

    const int ret = gpiod_line_set_value(line_, value);
    if (ret < 0) {
        throw std::runtime_error("Failed to set gpio value");
    }
}