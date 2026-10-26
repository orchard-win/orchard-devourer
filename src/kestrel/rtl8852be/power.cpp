// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Power and efuse sequences adapted from Linux rtw89 rtw8852b.c, mac.c and
// efuse.c. Copyright(c) 2019-2022 Realtek Corporation.
// Orchard adaptation 2026.
// Pinned source and deviations: docs/rtl8852be-pcie-implementation-map.md.
#include "power.h"

#include <bit>

namespace orchard::rtl8852be {
namespace {
constexpr std::uint16_t iso_ctrl = 0x0000, func_en = 0x0002, pw_ctrl = 0x0004,
                        wl_efuse_ctrl = 0x000a, swr_ctrl1 = 0x0010, adie_pad = 0x0018,
                        afe_ldo = 0x0020, efuse_ctrl = 0x0030, efuse_ctrl1 = 0x0038,
                        sdio_ctrl = 0x0070, hci_ldo = 0x007a, platform = 0x0088,
                        wllps = 0x0090, scoreboard = 0x00ac, pmc_dbg2 = 0x00cc,
                        sps_dig_on = 0x0200, sps_ana_on2 = 0x0228, xtal_si = 0x0270,
                        eesk_sel = 0x02d8, wlrf_ctrl = 0x02f0, power_state = 0x03f0,
                        sps_dig_off = 0x0400, dmac_func_en = 0x8400, cmac_func_en = 0xc000;
// SYS_PW_CTRL
constexpr std::uint32_t dis_pdnsusen_sopc = 1u << 18, rdy_syspwr = 1u << 17, en_wlon = 1u << 16,
                        apdm_hpdn = 1u << 15, afsm_pcie_sus = 1u << 12, afsm_wlsus = 1u << 11,
                        apfm_swlps = 1u << 10, apfm_offmac = 1u << 9, apfn_onmac = 1u << 8;
constexpr std::uint32_t pmc_wrmsk = 1u << 2, eb2core = 1u << 8, ev2ef_b14 = 1u << 14,
                        ev2ef_b15 = 1u << 15, wl_pta_1p3 = 1u << 6, wl_rfc_1p3 = 1u << 5;
// MAC_FUNC_EN through DMACREG_GCKEN, and the CMAC set pwr_on_func enables.
constexpr std::uint32_t dmac_all = 0x7fff8000, cmac_all = 0x7000803f;
constexpr std::uint32_t sps_ana_default = 0x4f42, sw_lps_option = 0x0001a0b2;
constexpr std::uint32_t mac_on = 1, notify_tp_major = 0x81, notify_pwr_major = 0x80;
// XTAL SI offsets and bits (core.h, reg.h).
constexpr std::uint8_t xmd2 = 0x24, xmd4 = 0x26, rfc_s0 = 0x80, rfc_s1 = 0x81, anapar_wl = 0x90,
                       sram_ctrl = 0xa1;
constexpr std::uint8_t pon_wei = 1u << 0, pon_ei = 1u << 1, off_wei = 1u << 2, off_ei = 1u << 3,
                       rfc2rf = 1u << 4, shdn_wl = 1u << 5, gnd_shdn_wl = 1u << 6,
                       sram2rfc = 1u << 7, sram_dis = 1u << 1, ldo_lps = 0x70, lps_cap = 0x0f,
                       rf00 = 1u << 0, rf10 = 1u << 0;

class Sequence {
public:
    explicit Sequence(MmioIo& io) : io_(io) {}
    PowerResult result;

    bool fail(PowerError error, std::uint16_t address) {
        if (result) { result.error = error; result.address = address; }
        return false;
    }
    bool read(unsigned width, std::uint16_t address, std::uint32_t& value) {
        if (!io_.read(width, address, value)) return fail(PowerError::read, address);
        if (!all_ones(width, value)) return true;
        std::uint32_t word = 0;
        if (width != 4 && io_.read(4, static_cast<std::uint16_t>(address & ~3u), word) &&
            !all_ones(4, word)) return true;
        return fail(PowerError::read, address);
    }
    bool write(unsigned width, std::uint16_t address, std::uint32_t value) {
        return io_.write(width, address, value) || fail(PowerError::write, address);
    }
    bool update(unsigned width, std::uint16_t address, std::uint32_t clear, std::uint32_t set) {
        std::uint32_t value = 0;
        return read(width, address, value) && write(width, address, (value & ~clear) | set);
    }
    bool set(unsigned width, std::uint16_t address, std::uint32_t bits) {
        return update(width, address, 0, bits);
    }
    bool clear(unsigned width, std::uint16_t address, std::uint32_t bits) {
        return update(width, address, bits, 0);
    }
    // rtw89_write*_mask: value is shifted into the mask.
    bool field(unsigned width, std::uint16_t address, std::uint32_t mask, std::uint32_t value) {
        return update(width, address, mask, (value << std::countr_zero(mask)) & mask);
    }
    bool pause(unsigned microseconds) {
        io_.delay_us(microseconds);
        return true;
    }
    // read_poll_timeout: read first, then sleep, one last read at the deadline.
    bool poll(std::uint16_t address, std::uint32_t mask, bool want_set, unsigned interval_us,
              unsigned timeout_us, std::uint32_t* last = nullptr) {
        for (unsigned elapsed = 0;; elapsed += interval_us) {
            std::uint32_t value = 0;
            if (!read(4, address, value)) return false;
            if (last) *last = value;
            if (((value & mask) != 0) == want_set) return true;
            if (elapsed >= timeout_us) return fail(PowerError::timeout, address);
            io_.delay_us(interval_us);
        }
    }
    // rtw89_mac_write_xtal_si_ax: a masked write to the XTAL serial interface.
    bool xtal(std::uint8_t offset, std::uint8_t value, std::uint8_t mask) {
        const std::uint32_t command = offset | (std::uint32_t{value} << 8) |
                                      (std::uint32_t{mask} << 16) | (1u << 31);
        std::uint32_t done = 0;
        if (!write(4, xtal_si, command)) return false;
        if (!poll(xtal_si, 1u << 31, false, 50, 50000, &done)) {
            if (result.error == PowerError::timeout) {
                result.error = PowerError::xtal_timeout;
                result.address = offset;
            }
            return false;
        }
        if ((done & 0xff) != offset || ((done >> 8) & 0xff) != value) ++result.xtal_mismatches;
        return true;
    }

private:
    MmioIo& io_;
};

bool sps_dig_off_setting(Sequence& s, std::uint8_t rfe) {
    if (rfe == 0x5)
        return s.field(4, sps_dig_off, 0x03, 1) && s.field(4, sps_dig_off, 0x0c, 1) &&
               s.field(4, sps_dig_off, 0x30, 2) && s.field(4, sps_dig_off, 0xc0, 1);
    return s.field(4, sps_dig_off, 0x03, 1) && s.field(4, sps_dig_off, 0x30, 3);
}

// rtw8852b_pwr_on_func on PCIe.
bool power_on_func(Sequence& s, const PowerConfig& config) {
    const bool mac =
        s.write(2, sps_ana_on2, sps_ana_default) &&
        s.clear(4, pw_ctrl, afsm_wlsus | afsm_pcie_sus) &&
        s.set(4, pw_ctrl, dis_pdnsusen_sopc) &&
        s.set(4, wllps, 1u << 1) && // DIS_WLBT_LPSEN_LOPC
        s.clear(4, pw_ctrl, apdm_hpdn) &&
        s.clear(4, pw_ctrl, apfm_swlps) &&
        s.poll(pw_ctrl, rdy_syspwr, true, 1000, 20000) &&
        s.set(4, afe_ldo, 1u << 23) && // AON_OFF_PC_EN
        s.poll(afe_ldo, 1u << 23, true, 1000, 20000) &&
        sps_dig_off_setting(s, config.rfe) &&
        s.set(4, pw_ctrl, en_wlon) &&
        s.set(4, pw_ctrl, apfn_onmac) &&
        s.poll(pw_ctrl, apfn_onmac, false, 1000, 20000) &&
        s.set(1, platform, 1) && s.clear(1, platform, 1) &&
        s.set(1, platform, 1) && s.clear(1, platform, 1) &&
        s.set(1, platform, 1) &&
        s.clear(4, sdio_ctrl, 1u << 12); // PCIE_CALIB_EN_V1
    const bool analog =
        mac &&
        s.set(4, adie_pad, wl_pta_1p3) &&
        s.xtal(anapar_wl, gnd_shdn_wl, gnd_shdn_wl) &&
        s.set(4, adie_pad, wl_rfc_1p3) &&
        s.xtal(anapar_wl, shdn_wl, shdn_wl) &&
        s.xtal(anapar_wl, off_wei, off_wei) &&
        s.xtal(anapar_wl, off_ei, off_ei) &&
        s.xtal(anapar_wl, 0, rfc2rf) &&
        s.xtal(anapar_wl, pon_wei, pon_wei) &&
        s.xtal(anapar_wl, pon_ei, pon_ei) &&
        s.xtal(anapar_wl, 0, sram2rfc) &&
        s.xtal(sram_ctrl, 0, sram_dis) &&
        s.xtal(xmd2, 0, ldo_lps) &&
        s.xtal(xmd4, 0, lps_cap) &&
        s.set(4, pmc_dbg2, pmc_wrmsk) &&
        s.set(4, iso_ctrl, eb2core) &&
        s.clear(4, iso_ctrl, ev2ef_b15) &&
        s.pause(1000) &&
        s.clear(4, iso_ctrl, ev2ef_b14) &&
        s.clear(4, pmc_dbg2, pmc_wrmsk);
    if (!analog) return false;
    // Voltage trim for a chip whose efuse was read and has no power-K. Linux
    // skips it on the probe's first power-on, when the efuse is not known yet.
    if (config.efuse_valid && !config.power_k_valid) {
        if (!s.field(4, sps_dig_on, 0x0000000f, 0x9) || // VOL_L1
            !s.field(4, sps_dig_on, 0x03c00000, 0xa))   // VREFPFM_L
            return false;
        if (config.cut == 1 && !(s.set(4, pmc_dbg2, pmc_wrmsk) &&
                                 s.field(2, hci_ldo, 0x0f, 0xa) && // R_AX_VADJ
                                 s.clear(4, pmc_dbg2, pmc_wrmsk)))
            return false;
    }
    return s.set(4, dmac_func_en, dmac_all) && s.set(4, cmac_func_en, cmac_all) &&
           s.field(4, eesk_sel, 0xf0, 1); // PINMUX_EESK_FUNC_SEL_BT_LOG
}

// rtw8852b_pwr_off_func on PCIe.
bool power_off_func(Sequence& s) {
    return s.write(2, sps_ana_on2, sps_ana_default) &&
           s.xtal(anapar_wl, rfc2rf, rfc2rf) &&
           s.xtal(anapar_wl, 0, off_ei) &&
           s.xtal(anapar_wl, 0, off_wei) &&
           s.xtal(rfc_s0, 0, rf00) &&
           s.xtal(rfc_s1, 0, rf10) &&
           s.xtal(anapar_wl, sram2rfc, sram2rfc) &&
           s.xtal(anapar_wl, 0, pon_ei) &&
           s.xtal(anapar_wl, 0, pon_wei) &&
           s.set(4, pw_ctrl, en_wlon) &&
           s.clear(4, wlrf_ctrl, 1u << 17) && // AFC_AFEDIG
           s.clear(1, func_en, 3) &&          // FEN_BB_GLB_RSTN | FEN_BBRSTB
           s.clear(4, adie_pad, wl_rfc_1p3) &&
           s.xtal(anapar_wl, 0, shdn_wl) &&
           s.clear(4, adie_pad, wl_pta_1p3) &&
           s.xtal(anapar_wl, 0, gnd_shdn_wl) &&
           s.set(4, pw_ctrl, apfm_offmac) &&
           s.poll(pw_ctrl, apfm_offmac, false, 1000, 20000) &&
           s.write(4, wllps, sw_lps_option) &&
           s.set(4, swr_ctrl1, 1u << 10) &&            // SYM_CTRL_SPS_PWMFREQ
           s.field(4, sps_dig_on, 0x00060000, 3) &&    // REG_ZCDC_H
           s.set(4, pw_ctrl, apfm_swlps);
}

// rtw89_enable/disable_efuse_pwr_cut_ddv. The burst mode is A-cut only.
bool efuse_power_cut(Sequence& s, unsigned cut, bool enable) {
    if (enable)
        return s.set(1, pmc_dbg2, pmc_wrmsk) && s.set(2, iso_ctrl, ev2ef_b14) &&
               s.pause(1000) && s.set(2, iso_ctrl, ev2ef_b15) && s.clear(2, iso_ctrl, eb2core) &&
               (cut != 0 || s.set(4, efuse_ctrl1, 1u << 19));
    return (cut != 0 || s.clear(4, efuse_ctrl1, 1u << 19)) &&
           s.set(2, iso_ctrl, eb2core) && s.clear(2, iso_ctrl, ev2ef_b15) && s.pause(1000) &&
           s.clear(2, iso_ctrl, ev2ef_b14) && s.clear(1, pmc_dbg2, pmc_wrmsk);
}

bool dump_once(Sequence& s, unsigned cut, std::uint16_t start, std::span<std::uint8_t> out) {
    if (!efuse_power_cut(s, cut, true)) return false;
    for (std::size_t i = 0; i < out.size(); ++i) {
        const auto address = static_cast<std::uint32_t>(start + i);
        std::uint32_t value = 0;
        if (!s.write(4, efuse_ctrl, (address << 16) & 0x07ff0000) || // EF_RDY clear
            !s.poll(efuse_ctrl, 1u << 29, true, 1, 1000000, &value))
            return false;
        out[i] = static_cast<std::uint8_t>(value);
    }
    return efuse_power_cut(s, cut, false);
}

// rtw89_dump_physical_efuse_map: five attempts, the last failure reported.
bool dump(Sequence& s, unsigned cut, std::uint16_t start, std::span<std::uint8_t> out) {
    for (int attempt = 0; attempt < 5; ++attempt) {
        s.result.error = PowerError::none;
        if (dump_once(s, cut, start, out)) return true;
    }
    return false;
}

bool power_switch_on(Sequence& s, const PowerConfig& config) {
    std::uint32_t state = 0;
    if (!s.read(4, power_state, state)) return false;
    if (((state >> 8) & 3) == mac_on) return s.fail(PowerError::mac_on, power_state);
    return power_on_func(s, config) && s.write(1, scoreboard + 3, notify_tp_major);
}
}

PowerResult power_on(MmioIo& io, const PowerConfig& config) {
    Sequence s(io);
    power_switch_on(s, config);
    return s.result;
}

PowerResult power_off(MmioIo& io) {
    Sequence s(io);
    if (power_off_func(s)) s.write(1, scoreboard + 3, notify_pwr_major);
    return s.result;
}

bool decode_efuse(std::span<const std::uint8_t> physical, std::span<std::uint8_t> logical) {
    constexpr std::size_t secure = 4; // sec_ctrl_efuse_size
    if (physical.size() < 2 * secure) return false;
    const std::size_t end = physical.size() - secure;
    for (std::size_t at = secure; at < end;) {
        const std::uint8_t first = physical[at], second = physical[at + 1];
        if (first == 0xff || second == 0xff) break;
        const std::size_t block = ((second & 0xf0u) >> 4) | ((first & 0x0fu) << 4);
        const unsigned words = second & 0x0fu; // a clear bit means the word follows
        at += 2;
        for (unsigned i = 0; i < 4; ++i) {
            if (words & (1u << i)) continue;
            const std::size_t to = (block << 3) + (i << 1);
            if (at + 1 > end - 1 || to + 1 > logical.size()) return false;
            logical[to] = physical[at];
            logical[to + 1] = physical[at + 1];
            at += 2;
        }
    }
    return true;
}

PowerResult read_efuse(MmioIo& io, unsigned cut, Efuse& efuse) {
    Sequence s(io);
    std::uint32_t autoload = 0;
    if (!s.read(2, wl_efuse_ctrl, autoload)) return s.result;
    efuse.autoload = autoload & (1u << 5); // AUTOLOAD_SUS
    if (!dump(s, cut, 0, efuse.physical)) return s.result;
    efuse.logical.fill(0xff);
    if (!decode_efuse(efuse.physical, efuse.logical)) {
        s.fail(PowerError::efuse_map, 0);
        return s.result;
    }
    dump(s, cut, phycap_address, efuse.phycap);
    return s.result;
}
}
