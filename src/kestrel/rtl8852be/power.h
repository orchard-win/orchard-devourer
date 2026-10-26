#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "rtl8852be.h"

// RTL8852BE power switch and efuse read, from Linux rtw89 at
// 734bef548a96e8774d6eef5dfa2026c6edc4e80b (rtw8852b.c, mac.c, efuse.c).
// See docs/rtl8852be-pcie-implementation-map.md for deviations.
namespace orchard::rtl8852be {
enum class PowerError { none, read, write, timeout, xtal_timeout, mac_on, efuse_map };
struct PowerResult {
    PowerError error = PowerError::none;
    std::uint16_t address = 0; // register, or XTAL SI offset for xtal_timeout
    unsigned xtal_mismatches = 0; // Linux only warns about these
    explicit operator bool() const { return error == PowerError::none; }
};

// What the efuse has said so far. Linux powers on once with all of it unknown
// (the defaults), reads the efuse, powers off, and powers on again with it.
struct PowerConfig {
    unsigned cut = 1;           // SYS_CFG1 CV; CBV PCIe gets the HCI LDO trim
    bool efuse_valid = false;   // autoload, not a valid MAC address
    bool power_k_valid = false;
    std::uint8_t rfe = 0;
};

// rtw89_mac_power_switch for one attempt. Refuses a MAC already on, as Linux
// does; the caller powers off and retries like rtw89_mac_pwr_on.
PowerResult power_on(MmioIo& io, const PowerConfig& config);
PowerResult power_off(MmioIo& io);

inline constexpr std::size_t physical_efuse_bytes = 1216, logical_efuse_bytes = 2048;
inline constexpr std::uint16_t phycap_address = 0x580;
inline constexpr std::size_t phycap_bytes = 128;

struct Efuse {
    bool autoload = false; // SYS_WL_EFUSE_CTRL bit5, Linux efuse.valid
    std::array<std::uint8_t, physical_efuse_bytes> physical{}; // as read
    std::array<std::uint8_t, logical_efuse_bytes> logical{};
    std::array<std::uint8_t, phycap_bytes> phycap{};
    std::uint8_t rfe() const { return logical[0x2ca]; }
    std::uint8_t xtal() const { return logical[0x2b9]; }
    std::span<const std::uint8_t, 6> mac() const { return std::span(logical).subspan<0x400, 6>(); }
    bool power_k() const { return phycap[0x5e9 - phycap_address] == 0xaa; }
    // External PN and customer bytes (0x5ec/0x5ed): firmware security selection.
    std::uint8_t external_pn() const { return phycap[0x5ec - phycap_address]; }
    std::uint8_t customer() const { return phycap[0x5ed - phycap_address]; }
};

// rtw89_parse_efuse_map_ax and the phycap dump, without the 96-byte DAV map
// (read over XTAL SI) that no 8852B field here uses. Needs the MAC powered on.
PowerResult read_efuse(MmioIo& io, unsigned cut, Efuse& efuse);
// Physical-to-logical efuse decoding, exposed for tests.
bool decode_efuse(std::span<const std::uint8_t> physical, std::span<std::uint8_t> logical);
}
