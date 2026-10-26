#include "pcie_control.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace be = orchard::rtl8852be;
namespace {
unsigned checks = 0;
void check(bool condition, const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
// Byte-addressed device model with independent DBI/MDIO engines and hardware
// self-clearing flags. Fault injection happens at the transport boundary.
struct Device final : be::PcieIo {
    std::array<std::uint8_t, 65536> mmio{};
    std::array<std::uint8_t, 4096> config{};
    std::array<std::array<std::uint16_t, 64>, 2> phy{};
    std::vector<std::tuple<bool, unsigned, std::uint16_t, std::uint32_t>> trace;
    unsigned fail_at = 0, ring_setups = 0, waited = 0;
    bool dbi_stuck = false, mdio_stuck = false, bdram_stuck = false, rings_fail = false;
    Device() {
        config[0x82] = 2; config[0x719] = 0x5a;
        phy[0][0x1b] = 0xa321; phy[0][0x1d] = 0x4510;
        phy[1][0x30] = 0x7123;
        put(4, 0x1000, 0x42800); put(4, 0x1004, 0xa0b055aa);
        put(4, 0x8410, 0x40000073); put(4, 0x8414, 0xa000b000);
        put(4, 0x9a00, 0x1234567a);
    }
    std::uint32_t get(unsigned width, std::uint16_t a) const {
        std::uint32_t value = 0;
        for (unsigned i = 0; i < width; ++i) value |= std::uint32_t{mmio[a+i]} << (8*i);
        return value;
    }
    void put(unsigned width, std::uint16_t a, std::uint32_t value) {
        for (unsigned i = 0; i < width; ++i) mmio[a+i] = static_cast<std::uint8_t>(value >> (8*i));
    }
    bool step(bool writing, unsigned width, std::uint16_t a, std::uint32_t value) {
        trace.emplace_back(writing, width, a, value);
        return trace.size() != fail_at;
    }
    bool read(unsigned width, std::uint16_t a, std::uint32_t& value) override {
        if (!step(false, width, a, 0)) return false;
        value = get(width, a); return true;
    }
    bool write(unsigned width, std::uint16_t a, std::uint32_t value) override {
        if (!step(true, width, a, value)) return false;
        put(width, a, value);
        if (a == 0x1092 && width == 1 && !dbi_stuck) {
            const auto control = get(2, 0x1090);
            const auto base = control & 0xffc;
            if (value == 2) for (unsigned i = 0; i < 4; ++i) mmio[0x1098+i] = config[base+i];
            if (value == 1) for (unsigned i = 0; i < 4; ++i)
                if (control & (1u << (12+i))) config[base+i] = mmio[0x1094+i];
            put(1, 0x1092, 0);
        }
        if (a == 0x10a0 && width == 2 && (value & 0x300) && !mdio_stuck) {
            auto& reg = phy[(value >> 13) & 1][(value & 0x1f) + ((value & 0x1000) ? 32 : 0)];
            if (value & 0x100) reg = static_cast<std::uint16_t>(get(2, 0x10a4));
            if (value & 0x200) put(2, 0x10a6, reg);
            put(2, 0x10a0, value & ~0x300u);
        }
        if (a == 0x1000 && width == 4 && !bdram_stuck) put(4, a, value & ~8u);
        return true;
    }
    void delay_us(unsigned us) override { waited += us; }
    bool setup_rings() override {
        ++ring_setups;
        check((get(4, 0x1000) & 0x2800) == 0 && (get(4, 0x1010) & 0x100000), "rings only programmed while HCI/IO stopped");
        check(get(4, 0x101c) == 0, "rings only programmed after idle");
        return !rings_fail;
    }
    bool touched(std::uint16_t a) const {
        return std::any_of(trace.begin(), trace.end(), [a](auto op) { return std::get<2>(op) == a; });
    }
};
void lifecycle() {
    Device d;
    be::PcieControl control(d);
    check(bool(control.prepare()), "BE preparation succeeds");
    check(d.phy[0][0x1b] == 0x1321 && d.phy[0][0x1d] == 0x451c, "BE DPHY delay and autok-x preserve other PHY bits");
    check(d.phy[1][0x30] == 0x5123 && d.config[0x719] == 0x5a, "Gen2 refclk disabled, original L1BD restored");
    check(d.get(4, 0x1010) == 0x000b0f00, "only CH12 open, WPDMA stopped during FWDL");
    check(d.get(4, 0x1000) == 0x00c0ff00, "B HCI, burst, packet mode and reset-keep golden value");
    check(d.get(4, 0x1004) == 0xa1b155aa, "B WD intervals preserve unrelated fields");
    check(d.get(4, 0x13f0) == 0x70001 && d.get(4, 0x11d8) == 0x83, "B tags and watchdog");
    check(d.get(4, 0x8810) == 1 && d.get(4, 0x9a00) == 0x12345678, "old eight-byte address format selected in dispatcher and PKTIN");
    check(d.touched(0x13b0) && !d.touched(0x13b8) && !d.touched(0x3000) && !d.touched(0x1600), "no CE register bank used");
    check(bool(control.start(false)), "host-forbidden LTR allows DMA without requests");
    check(d.get(4, 0x1010) == 0 && d.get(4, 0x8410) == 0x40000000, "all TX open and LTR generators/requests silenced");
    check(!d.touched(0x8418), "host-forbidden LTR never programs latency/request path");
    check(bool(control.stop()) && !(d.get(4, 0x1000) & 0x2800), "DMA stopped");
    check(d.get(4, 0x1014) == 0x70f && d.get(4, 0x1018) == 3, "only B index banks reset");

    Device allowed;
    be::PcieControl yes(allowed);
    check(bool(yes.prepare()) && bool(yes.start(true)), "host-permitted LTR start");
    check(allowed.get(4, 0x8410) == 0x40002743 && allowed.get(4, 0x8414) == 0xa028b028, "B LTR fields and preserved bits");
    check(allowed.get(4, 0x8418) == 0x90039003 && allowed.get(4, 0x841c) == 0x880b880b, "B LTR latency values");
    check(bool(yes.stop()) && !(allowed.get(4, 0x8410) & 0x73), "shutdown does not leave LTR generation armed");
    Device gen1; gen1.config[0x82] = 1; gen1.phy[0][0x30] = 0x6123;
    check(bool(be::PcieControl(gen1).prepare()) && gen1.phy[0][0x30] == 0x4123 && gen1.phy[1][0x30] == 0x7123, "Gen1 link selects its own MDIO page");
}
void all_ones() {
    Device byte; byte.config[0x719] = 0xff;
    check(bool(be::PcieControl(byte).prepare()) && byte.config[0x719] == 0xff,
          "all-ones config byte inside a live dword is register content");
    Device lost;
    for (unsigned i = 0x80; i < 0x84; ++i) lost.config[i] = 0xff;
    const auto result = be::PcieControl(lost).prepare();
    check(!result && result.error == be::PcieError::read && result.address == 0x109a,
          "all-ones byte inside an all-ones dword is a lost device");
    Device dead; dead.put(4, 0x101c, 0xffffffff);
    check(!be::PcieControl(dead).stop() && !dead.touched(0x1014), "all-ones busy dword is not idle");
}
void failures() {
    Device baseline;
    check(bool(be::PcieControl(baseline).prepare()), "baseline preparation");
    const auto calls = baseline.trace.size();
    for (unsigned failure = 1; failure <= calls; ++failure) {
        Device d; d.fail_at = failure;
        const auto result = be::PcieControl(d).prepare();
        check(!result, "every preparation I/O error propagates, including restore errors");
        check(!(d.get(4, 0x1000) & 0x2800) && (d.get(4, 0x1010) & 0x100000), "preparation failure stops DMA");
        check(!(d.get(4, 0x8410) & 0x73), "preparation failure silences LTR");
        const auto failed = baseline.trace[failure-1];
        if (!std::get<0>(failed) && std::get<2>(failed) == 0x10a6 && failure > 50)
            check(result.error == be::PcieError::read && result.address == 0x10a6 && d.config[0x719] == 0x5a,
                  "successful L1BD restore does not hide original MDIO read error");
    }
    Device start_baseline;
    check(bool(be::PcieControl(start_baseline).start(true)), "start baseline");
    for (unsigned failure = 1; failure <= start_baseline.trace.size(); ++failure) {
        Device d; d.fail_at = failure;
        check(!be::PcieControl(d).start(true), "LTR and channel-open failure propagates");
        check(!(d.get(4, 0x1000) & 0x2800) && (d.get(4, 0x1010) & 0x100000), "failed start leaves DMA stopped");
    }
    Device busy; busy.put(4, 0x101c, 1);
    auto result = be::PcieControl(busy).prepare();
    check(!result && result.error == be::PcieError::timeout && result.cleanup_failed, "busy DMA failure and cleanup failure both reported");
    check(!busy.touched(0x1014) && !busy.touched(0x1018) && busy.ring_setups == 0, "busy DMA never clears indices or programs rings");
    check(busy.waited == 20000, "both idle attempts bounded");
    Device unavailable; unavailable.put(4, 0x101c, 0xeaeaeaea);
    check(!be::PcieControl(unavailable).stop() && !unavailable.touched(0x1014), "unreadable busy register is not idle");
    Device dbi; dbi.dbi_stuck = true;
    result = be::PcieControl(dbi).prepare();
    check(!result && result.address == 0x1092 && dbi.waited == 200, "DBI timeout bounded");
    Device mdio; mdio.mdio_stuck = true;
    result = be::PcieControl(mdio).prepare();
    check(!result && result.address == 0x10a0 && mdio.waited == 2000, "MDIO timeout bounded");
    Device bdram; bdram.bdram_stuck = true;
    result = be::PcieControl(bdram).prepare();
    check(!result && result.address == 0x1000 && bdram.waited == 10000, "BDRAM timeout bounded and no DMA started");
    Device ring; ring.rings_fail = true;
    check(be::PcieControl(ring).prepare().error == be::PcieError::ring_setup, "ring setup failure propagated");
    Device speed; speed.config[0x82] = 3;
    check(be::PcieControl(speed).prepare().error == be::PcieError::link_speed, "unexpected link generation refused");
}
}
int main() {
    try { lifecycle(); all_ones(); failures(); std::cout << checks << " BE PCIe lifecycle/failure checks passed\n"; return 0; }
    catch (const std::exception& e) { std::cerr << "FAILED after " << checks << ": " << e.what() << '\n'; return 1; }
}
