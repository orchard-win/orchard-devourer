#pragma once
#include "pcie_control.h"
#include "kestrel_pta.h"

namespace orchard::rtl8852be {
class KestrelPcie final : public PcieIo {
public:
    explicit KestrelPcie(RtlAdapter& device) : device_(device), registers_(device) {}
    bool read(unsigned width, std::uint16_t address, std::uint32_t& value) override {
        return registers_.read(width, address, value);
    }
    bool write(unsigned width, std::uint16_t address, std::uint32_t value) override {
        return registers_.write(width, address, value);
    }
    void delay_us(unsigned microseconds) override {
        devourer::precise_delay_us(microseconds);
    }
    bool setup_rings() override {
        try { device_.hci_setup(); return true; }
        catch (const std::exception&) { return false; }
    }
private:
    RtlAdapter& device_;
    KestrelPta registers_;
};
}
