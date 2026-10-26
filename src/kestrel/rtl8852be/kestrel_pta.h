#pragma once

#include "rtl8852be.h"
#include "RtlAdapter.h"
#include "PreciseDelay.h"
#include <exception>

namespace orchard::rtl8852be {
class KestrelPta final : public Registers {
public:
    explicit KestrelPta(RtlAdapter& device) : device_(device) {}
    bool read(unsigned width, std::uint16_t address, std::uint32_t& value) override {
        if (!raw_read(width, address, value)) return false;
        if (!all_ones(width, value)) return true;
        std::uint32_t word = 0;
        return width != 4 && raw_read(4, static_cast<std::uint16_t>(address & ~3u), word) &&
               !all_ones(4, word);
    }
    bool write(unsigned width, std::uint16_t address, std::uint32_t value) override {
        try {
        switch (width) {
        case 1: return device_.rtw_write8(address, static_cast<std::uint8_t>(value));
        case 2: return device_.rtw_write16(address, static_cast<std::uint16_t>(value));
        case 4: return device_.rtw_write32(address, value);
        default: return false;
        }
        } catch (const std::exception&) { return false; }
    }
    bool read_lte(std::uint16_t address, std::uint32_t& value) override {
        return ready() && write(4, 0xdaf0, 0x800f0000u | address) &&
               ready() && read(4, 0xdaf8, value);
    }
    bool write_lte(std::uint16_t address, std::uint32_t value) override {
        // Wait for the final command too, including the last write at init.
        return ready() && write(4, 0xdaf4, value) &&
               write(4, 0xdaf0, 0xc00f0000u | address) && ready();
    }
private:
    bool raw_read(unsigned width, std::uint16_t address, std::uint32_t& value) {
        try {
        switch (width) {
        case 1: value = device_.rtw_read8(address); return true;
        case 2: value = device_.rtw_read16(address); return true;
        case 4: value = device_.rtw_read32(address); return true;
        default: return false;
        }
        } catch (const std::exception&) { return false; }
    }
    bool ready() {
        for (unsigned i = 0; i < 1000; ++i) {
            std::uint32_t value = 0;
            if (!read(1, 0xdaf3, value)) return false;
            if (value & 0x20) return true;
            devourer::precise_delay_us(50);
        }
        return false;
    }
    RtlAdapter& device_;
};
}
