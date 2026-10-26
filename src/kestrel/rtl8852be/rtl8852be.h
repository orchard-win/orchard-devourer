#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

// RTL8852B PCIe contracts from Linux rtw89 at
// 734bef548a96e8774d6eef5dfa2026c6edc4e80b. See docs/rtl8852be-pcie-implementation-map.md.
namespace orchard::rtl8852be {
struct TxRing {
    std::uint16_t count, index, bdram, base;
    std::uint8_t start, maximum, minimum;
};
inline constexpr std::array<TxRing, 13> tx_rings{{
    {0x1024, 0x1058, 0x1200, 0x1110, 0, 5, 2},
    {0x1026, 0x105c, 0x1204, 0x1118, 5, 5, 2},
    {0x1028, 0x1060, 0x1208, 0x1120, 10, 5, 2},
    {0x102a, 0x1064, 0x120c, 0x1128, 15, 5, 2},
    {}, {}, {}, {},
    {0x1034, 0x1078, 0x1220, 0x1150, 20, 4, 1},
    {0x1036, 0x107c, 0x1224, 0x1158, 24, 4, 1},
    {}, {},
    {0x1038, 0x1080, 0x1228, 0x1160, 28, 4, 1},
}};
inline constexpr unsigned wd_body_bytes = 24, driver_info_bytes = 8;
inline constexpr unsigned firmware_precost = 40;
// Windows regproxy cannot enable PCI config 0x719[5] (DAC). Until that is
// implemented, *every* BE DMA allocation must fit below 4 GiB, not only TX data.
inline constexpr unsigned dma_address_bits = 32;
inline constexpr std::uint32_t active_stop_mask = 0x00070f00;
inline constexpr std::uint32_t active_index_mask = 0x0000070f;
constexpr bool active_channel(unsigned channel) {
    return channel < tx_rings.size() && tx_rings[channel].count != 0;
}
constexpr bool valid_dma(std::uint64_t address, std::uint64_t bytes) {
    constexpr auto limit = std::uint64_t{1} << dma_address_bits;
    return bytes != 0 && address < limit && bytes <= limit - address;
}
// What a function that stopped answering returns (0xeaeaeaea: unpowered
// domain). A narrow all-ones value can still be real register content, so
// callers confirm it against the aligned dword before calling it a failure.
constexpr bool all_ones(unsigned width, std::uint32_t value) {
    if (width == 4) return value == 0xffffffff || value == 0xeaeaeaea;
    return value == (width == 2 ? 0xffffu : 0xffu);
}

// Convert a B USB-style WD+frame to a PCIe WD page. No output is modified on
// invalid input. Caller retains the page and payload until an RPQ release.
// Return the populated page size (40 or 64), or zero on failure.
std::size_t make_wd_page(std::span<const std::uint8_t> source,
                         std::uint16_t sequence, std::uint64_t payload_dma,
                         std::span<std::uint8_t> page,
                         std::span<std::uint8_t> payload);

// Width is explicit: several coex registers are byte/word registers. LTE is
// a separate indirect address space; a failed/pending access must return false.
struct Registers {
    virtual ~Registers() = default;
    virtual bool read(unsigned width, std::uint16_t address, std::uint32_t& value) = 0;
    virtual bool write(unsigned width, std::uint16_t address, std::uint32_t value) = 0;
    virtual bool read_lte(std::uint16_t address, std::uint32_t& value) = 0;
    virtual bool write_lte(std::uint16_t address, std::uint32_t value) = 0;
};

// MMIO with delays, for the PCIe and power sequences. Unlike Registers,
// callers check reads for a lost dword themselves.
struct MmioIo {
    virtual ~MmioIo() = default;
    virtual bool read(unsigned width, std::uint16_t address, std::uint32_t& value) = 0;
    virtual bool write(unsigned width, std::uint16_t address, std::uint32_t value) = 0;
    virtual void delay_us(unsigned microseconds) = 0;
};

// First contact with a card: always-on system registers, read before any
// power-on or DMA. Names follow devourer src/kestrel/MacRegAx.h.
struct Identity {
    std::uint32_t iso_ctrl = 0, power_control = 0, platform = 0, config1 = 0,
                  status1 = 0, chip_info = 0, power_state = 0;
    std::uint8_t die() const { return static_cast<std::uint8_t>(chip_info); }
    unsigned cut() const { return (config1 >> 12) & 0xf; } // 0 = A-cut
    unsigned mac_power() const { return (power_state >> 8) & 3; } // 0 = off
    bool rtl8852b() const { return die() == 0x51; }
};
// Reads only. False on the first failed read or lost dword.
bool read_identity(Registers& registers, Identity& identity);

// Experimental MAC/PTA portion only. Firmware BTC policy, RF calibration
// handoff, scoreboard and efuse-dependent RF LUTs are still required for full
// coexistence. Do not advertise this as verified simultaneous Wi-Fi/BLE.
bool initialize_hardware_pta(Registers& registers);
bool set_pta_band(Registers& registers, bool five_ghz);
bool release_pta_to_bluetooth(Registers& registers);

// Control-thread operations; TX may read transmit_allowed() concurrently.
// A failed session cannot be restarted. Cleanup may be retried after I/O
// recovers, and never re-enables TX. This is MAC/PTA, not full firmware BTC.
class PtaSession {
public:
    bool initialize(Registers& registers);
    bool set_band(Registers& registers, bool five_ghz);
    bool shutdown(Registers& registers);
    void fault(Registers& registers);
    bool transmit_allowed() const { return state_.load() == State::active; }
    bool cleanup_pending() const { return cleanup_pending_; }
private:
    enum class State { dormant, active, updating, faulted, stopped };
    std::atomic<State> state_{State::dormant};
    bool cleanup_pending_ = false;
};
}
