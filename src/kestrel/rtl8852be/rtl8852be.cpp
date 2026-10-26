#include "rtl8852be.h"

#include <algorithm>

namespace orchard::rtl8852be {
namespace {
std::uint32_t get32(const std::uint8_t* p) {
    return std::uint32_t{p[0]} | (std::uint32_t{p[1]} << 8) |
           (std::uint32_t{p[2]} << 16) | (std::uint32_t{p[3]} << 24);
}
void put16(std::uint8_t* p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}
void put32(std::uint8_t* p, std::uint32_t value) {
    put16(p, static_cast<std::uint16_t>(value));
    put16(p + 2, static_cast<std::uint16_t>(value >> 16));
}
bool modify(Registers& io, unsigned width, std::uint16_t address,
            std::uint32_t clear, std::uint32_t set) {
    std::uint32_t value = 0;
    return io.read(width, address, value) &&
           io.write(width, address, (value & ~clear) | set);
}
}

std::size_t make_wd_page(std::span<const std::uint8_t> source,
                         std::uint16_t sequence, std::uint64_t payload_dma,
                         std::span<std::uint8_t> page,
                         std::span<std::uint8_t> payload) {
    if (source.size() < wd_body_bytes || sequence >= 512) return 0;
    const auto dw0 = get32(source.data());
    const std::size_t header = wd_body_bytes + ((dw0 & (1u << 22)) ? 24 : 0);
    if (source.size() <= header) return 0;
    const auto length = source.size() - header;
    const auto page_size = header + 8 + 8; // wp_info + old addr_info
    if (length > 4096 || length > payload.size() || page_size > page.size() ||
        !valid_dma(payload_dma, length)) return 0;
    std::fill(page.begin(), page.end(), std::uint8_t{0});
    std::copy_n(source.begin(), header, page.begin());
    std::copy(source.begin() + header, source.end(), payload.begin());
    put32(page.data(), (dw0 | (1u << 7)) & ~(1u << 10)); // WD_PAGE, !STF
    put32(page.data() + 4, (get32(page.data() + 4) & ~(0x3fu << 26)) | (1u << 26));
    put16(page.data() + header, static_cast<std::uint16_t>(sequence | 0x8000));
    auto* address = page.data() + header + 8;
    put16(address, static_cast<std::uint16_t>(length));
    put16(address + 2, 0x8001); // LS | ADDR_INFO_NUM=1; upper DMA bits zero
    put32(address + 4, static_cast<std::uint32_t>(payload_dma));
    return page_size;
}

bool read_identity(Registers& io, Identity& id) {
    struct Field { std::uint16_t address; std::uint32_t Identity::*field; };
    constexpr Field fields[] = {
        {0x0000, &Identity::iso_ctrl},    {0x0004, &Identity::power_control},
        {0x0088, &Identity::platform},    {0x00f0, &Identity::config1},
        {0x00f4, &Identity::status1},     {0x00fc, &Identity::chip_info},
        {0x03f0, &Identity::power_state}};
    for (const auto& [address, field] : fields) {
        std::uint32_t value = 0;
        if (!io.read(4, address, value) || all_ones(4, value)) return false;
        id.*field = value;
    }
    return true;
}

bool release_pta_to_bluetooth(Registers& io) {
    // BTC_ANT_WOFF: PTA owner BT; do not toggle the USB Bluetooth function.
    return modify(io, 1, 0x0073, 1u << 2, 0);
}

bool set_pta_band(Registers& io, bool five_ghz) {
    std::uint32_t old38 = 0, old3c = 0, old_plt = 0;
    if (!io.read_lte(0x0038, old38) || !io.read_lte(0x003c, old3c) ||
        !io.read(2, 0xc67c, old_plt)) return false;
    // Vendor mac_cfg_gnt_8852b, both bands. 2.4 GHz: both HW; 5 GHz:
    // WL software-high / BT HW. Preserve only WL_RX_CTRL, never BT forced-low.
    if (io.write_lte(0x0038, five_ghz ? 0x33003300u : 0u) &&
        io.write_lte(0x003c, (old3c & 0x100u) | (five_ghz ? 0xf0u : 0u)) &&
        io.write(2, 0xc67c, five_ghz ? 0x100 : 0x166)) return true;
    // Even a failed write can have reached the device. Attempt every restore;
    // PtaSession additionally faults TX and neutralizes grants on any failure.
    io.write_lte(0x0038, old38);
    io.write_lte(0x003c, old3c);
    io.write(2, 0xc67c, old_plt);
    return false;
}

bool initialize_hardware_pta(Registers& io) {
    // mac_coex_init: internal direction and RTK PTA mode. Kept separate from
    // firmware BTC init: register setup alone is not an arbitration policy.
    const bool ok =
        modify(io, 1, 0x0040, 0, 1u << 5) && // ENBT before LTE indirect access
        set_pta_band(io, false) &&
        modify(io, 1, 0xda20, 0, 1u << 1) &&
        modify(io, 1, 0xda35, 0, 1u) &&
        modify(io, 1, 0xda40, 0, (1u << 2) | (1u << 3)) &&
        modify(io, 1, 0xda42, 0, 1u) &&
        modify(io, 1, 0xcc07, 1u << 1, 0) &&
        modify(io, 2, 0xc340, 1u << 9, 1u << 5) &&
        modify(io, 1, 0x0040, 0xc0, 0) &&
        modify(io, 1, 0xda4c, 0, 1u) &&
        modify(io, 1, 0xda6c, 0x3f, 5) &&
        modify(io, 1, 0x0041, 1u << 2, 1u << 1) &&
        modify(io, 1, 0x0073, 0, 1u << 2);
    if (!ok) release_pta_to_bluetooth(io); // best effort; original failure survives
    return ok;
}

bool PtaSession::initialize(Registers& io) {
    if (state_ != State::dormant) return false;
    cleanup_pending_ = true;
    if (!initialize_hardware_pta(io)) { fault(io); return false; }
    state_ = State::active;
    return true;
}
bool PtaSession::set_band(Registers& io, bool five_ghz) {
    if (state_ != State::active) return false;
    state_ = State::updating;
    if (!set_pta_band(io, five_ghz)) { fault(io); return false; }
    state_ = State::active;
    return true;
}
bool PtaSession::shutdown(Registers& io) {
    if (state_ == State::active) state_ = State::stopped;
    if (!cleanup_pending_) return true;
    // WOFF: disable PHY0 priority masking and return owner to BT. Remove our
    // forced WL grants too; do not restore pre-init Wi-Fi-only BT-low grants.
    // Never touch CMAC1 or reset/power down the independent USB BT function.
    const bool plt = io.write(2, 0xc67c, 0x100);
    std::uint32_t old3c = 0;
    const bool read = io.read_lte(0x3c, old3c);
    const bool grant38 = io.write_lte(0x38, 0);
    const bool grant3c = read && io.write_lte(0x3c, old3c & 0x100);
    const bool owner = release_pta_to_bluetooth(io);
    cleanup_pending_ = !(plt && grant38 && grant3c && owner);
    if (cleanup_pending_) state_ = State::faulted;
    return !cleanup_pending_;
}
void PtaSession::fault(Registers& io) {
    state_ = State::faulted;
    shutdown(io);
}
}
