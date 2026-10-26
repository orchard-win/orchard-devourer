#pragma once

#include <cstdint>

#include "rtl8852be.h"

namespace orchard::rtl8852be {
struct PcieIo : MmioIo {
    // Called only after hardware DMA is idle. Failure must not start DMA.
    virtual bool setup_rings() = 0;
};

enum class PcieError { none, read, write, timeout, link_speed, ring_setup };
struct PcieResult {
    PcieError error = PcieError::none;
    std::uint16_t address = 0;
    bool cleanup_failed = false;
    explicit operator bool() const { return error == PcieError::none; }
};

// RTL8852BE only. Power/efuse and Windows proxy ownership are prerequisites;
// these register sequences do not themselves authorize freeing DMA memory.
class PcieControl {
public:
    explicit PcieControl(PcieIo& io) : io_(io) {}
    PcieResult prepare();
    PcieResult start(bool host_allows_ltr);
    PcieResult stop();
private:
    PcieIo& io_;
    PcieResult result_{};
    bool fail(PcieError error, std::uint16_t address);
    bool read(unsigned width, std::uint16_t address, std::uint32_t& value);
    bool write(unsigned width, std::uint16_t address, std::uint32_t value);
    bool modify(unsigned width, std::uint16_t address, std::uint32_t clear, std::uint32_t set);
    bool poll(unsigned width, std::uint16_t address, std::uint32_t mask, unsigned timeout_us);
    bool dbi(std::uint16_t address, std::uint32_t& value, bool writing);
    bool mdio(std::uint8_t address, bool gen2, std::uint32_t& value, bool writing);
    bool mdio_modify(std::uint8_t address, bool gen2, std::uint32_t clear, std::uint32_t set);
    bool disable_refclk();
    bool phy();
    bool mode();
    bool ltr(bool enable);
    bool stop_dma();
    bool clear_indices();
    bool prepare_impl();
    PcieResult finish(bool success);
};
}
