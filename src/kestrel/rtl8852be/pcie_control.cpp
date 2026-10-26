// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Register sequences adapted from Linux rtw89 pci.c / rtw8852be.c.
// Copyright(c) 2020 Realtek Corporation (pci.c).
// Copyright(c) 2020-2022 Realtek Corporation (rtw8852be.c).
// Orchard adaptation 2026.
// Pinned source and deviations: docs/rtl8852be-pcie-implementation-map.md.
#include "pcie_control.h"
#include "rtl8852be.h"

namespace orchard::rtl8852be {
namespace {
constexpr std::uint32_t io_stop = 1u << 20, wp_stop = 1u << 19;
constexpr std::uint32_t hci_enable = (1u << 11) | (1u << 13);
constexpr std::uint32_t ltr_enable = 0x43, ltr_requests = 0x30;
}
bool PcieControl::fail(PcieError error, std::uint16_t address) {
    if (result_) { result_.error = error; result_.address = address; }
    return false;
}
bool PcieControl::read(unsigned width, std::uint16_t address, std::uint32_t& value) {
    if (!io_.read(width, address, value)) return fail(PcieError::read, address);
    if (!all_ones(width, value)) return true;
    std::uint32_t word = 0;
    if (width != 4 && io_.read(4, static_cast<std::uint16_t>(address & ~3u), word) &&
        !all_ones(4, word)) return true;
    return fail(PcieError::read, address);
}
bool PcieControl::write(unsigned width, std::uint16_t address, std::uint32_t value) {
    return io_.write(width, address, value) || fail(PcieError::write, address);
}
bool PcieControl::modify(unsigned width, std::uint16_t address,
                         std::uint32_t clear, std::uint32_t set) {
    std::uint32_t value = 0;
    return read(width, address, value) && write(width, address, (value & ~clear) | set);
}
bool PcieControl::poll(unsigned width, std::uint16_t address,
                       std::uint32_t mask, unsigned timeout_us) {
    for (unsigned elapsed = 0; elapsed <= timeout_us; elapsed += 10) {
        std::uint32_t value = 0;
        if (!read(width, address, value)) return false;
        if (!(value & mask)) return true;
        if (elapsed != timeout_us) io_.delay_us(10);
    }
    return fail(PcieError::timeout, address);
}
bool PcieControl::dbi(std::uint16_t address, std::uint32_t& value, bool writing) {
    const auto lane = address & 3;
    if (writing && !write(1, static_cast<std::uint16_t>(0x1094 + lane), value)) return false;
    if (!write(2, 0x1090, (address & 0xffc) | (writing ? (1u << lane) << 12 : 0)) ||
        !write(1, 0x1092, writing ? 1 : 2) || !poll(1, 0x1092, 0xff, 200)) return false;
    return writing || read(1, static_cast<std::uint16_t>(0x1098 + lane), value);
}
bool PcieControl::mdio(std::uint8_t address, bool gen2, std::uint32_t& value, bool writing) {
    if (writing && !write(2, 0x10a4, value)) return false;
    const unsigned page = (gen2 ? 2 : 0) + (address >= 0x20 ? 1 : 0);
    const unsigned flag = writing ? 0x100 : 0x200;
    if (!write(1, 0x10a0, address & 0x1f) ||
        !modify(2, 0x10a0, 0x3000, page << 12) ||
        !modify(2, 0x10a0, 0, flag) || !poll(2, 0x10a0, flag, 2000)) return false;
    return writing || read(2, 0x10a6, value);
}
bool PcieControl::mdio_modify(std::uint8_t address, bool gen2,
                              std::uint32_t clear, std::uint32_t set) {
    std::uint32_t value = 0;
    if (!mdio(address, gen2, value, false)) return false;
    value = (value & ~clear) | set;
    return mdio(address, gen2, value, true);
}
bool PcieControl::disable_refclk() {
    std::uint32_t speed = 0, saved = 0;
    if (!dbi(0x82, speed, false)) return false;
    speed &= 3;
    if (speed != 1 && speed != 2) return fail(PcieError::link_speed, 0x82);
    if (!dbi(0x719, saved, false)) return false;
    bool ok = true;
    if (saved & 8) {
        auto disabled = saved & ~8u;
        ok = dbi(0x719, disabled, true);
    }
    if (ok) ok = mdio_modify(0x30, speed == 2, 1u << 13, 0);
    // A failed DBI write may have reached hardware. Restore on every attempted
    // override; a successful restore must never erase the primary MDIO error.
    if (saved & 8) {
        PcieControl cleanup(io_);
        if (!cleanup.dbi(0x719, saved, true)) {
            if (ok) result_ = cleanup.result_;
            result_.cleanup_failed = true;
            ok = false;
        }
    }
    return ok;
}
bool PcieControl::phy() {
    return modify(4, 0x1008, 1u << 5, 0) &&
        modify(4, 0x0004, 1u << 14, 0) &&
        modify(4, 0x0070, 1u << 14, 1u << 15) &&
        mdio_modify(0x1b, false, 0xf000, 0x1000) &&
        mdio_modify(0x1d, false, 0x000c, 0x000c) && disable_refclk() &&
        modify(4, 0x0074, 0, 1u << 5) && modify(4, 0x13f0, 1u << 4, 0) &&
        modify(4, 0x11d8, 0xf0, 0x83) && modify(4, 0x11c0, 0, 3) &&
        modify(4, 0x13f0, 3, 1) && modify(4, 0x1000, 0, 0xc00000);
}
bool PcieControl::mode() {
    return modify(4, 0x1000, (1u << 18) | 0x700 | 0x1c000, 0xd700) &&
        modify(4, 0x13f0, 0x70000, 0x70000) &&
        modify(4, 0x1004, 0x0f0f0000, 0x01010000) &&
        modify(4, 0x8810, 0, 1) && modify(4, 0x9a00, 1u << 1, 0);
}
bool PcieControl::ltr(bool enable) {
    if (!enable) return modify(4, 0x8410, ltr_enable | ltr_requests, 0);
    std::uint32_t ctrl = 0, threshold = 0, unused = 0;
    if (!read(4, 0x8410, ctrl) || !read(4, 0x8414, threshold) ||
        !read(4, 0x8418, unused) || !read(4, 0x841c, unused)) return false;
    // Configure while disabled, then enable last. Never generate a request
    // when DEVCTL2 says the upstream path does not support LTR.
    return write(4, 0x8410, ctrl & ~(ltr_enable | ltr_requests)) &&
        write(4, 0x8414, (threshold & ~0x0fff0fffu) | 0x00280028) &&
        write(4, 0x8418, 0x90039003) && write(4, 0x841c, 0x880b880b) &&
        write(4, 0x8410, (ctrl & ~(0x3700u | ltr_requests)) | 0x2743);
}
bool PcieControl::stop_dma() {
    const bool stopped = modify(4, 0x1010, 0, io_stop | wp_stop | active_stop_mask);
    const bool disabled = modify(4, 0x1000, hci_enable, 0);
    return stopped && disabled && poll(4, 0x101c, active_stop_mask | 3, 10000);
}
bool PcieControl::clear_indices() {
    return write(4, 0x1014, active_index_mask) && write(4, 0x1018, 3);
}
bool PcieControl::prepare_impl() {
    // LTR stays silent even when the host allows it: start() supplies the
    // thresholds once the rings are programmed.
    if (!ltr(false) || !phy() || !stop_dma() || !clear_indices() || !mode()) return false;
    if (!write(4, 0x01a0, 0) || !write(4, 0x10b0, 0) || !write(4, 0x13b0, 0)) return false;
    if (!io_.setup_rings()) return fail(PcieError::ring_setup, 0);
    return modify(4, 0x1000, 0, 8) && poll(4, 0x1000, 8, 10000) &&
        modify(4, 0x1010, active_stop_mask, active_stop_mask & ~(1u << 18)) &&
        modify(4, 0x1000, 0, hci_enable) && modify(4, 0x1010, io_stop, 0);
}
PcieResult PcieControl::finish(bool success) {
    if (!success) {
        PcieControl cleanup(io_);
        if (!cleanup.stop()) result_.cleanup_failed = true;
    }
    return result_;
}
PcieResult PcieControl::prepare() {
    result_ = {};
    return finish(prepare_impl());
}
PcieResult PcieControl::start(bool host_allows_ltr) {
    result_ = {};
    return finish(ltr(host_allows_ltr) && modify(4, 0x1010, active_stop_mask | wp_stop | io_stop, 0));
}
PcieResult PcieControl::stop() {
    result_ = {};
    // Even if LTR silencing fails, attempt to stop DMA; never reset indices
    // on a busy/unreadable device or after a failed stop write.
    ltr(false);
    if (stop_dma()) clear_indices();
    return result_;
}
}
