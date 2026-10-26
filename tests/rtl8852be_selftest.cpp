#include "rtl8852be.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <map>
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
struct FakeRegisters final : be::Registers {
    std::map<std::uint16_t, std::uint32_t> mmio, lte;
    std::vector<std::tuple<unsigned, std::uint16_t, std::uint32_t>> writes;
    unsigned calls = 0, fail_at = 0;
    bool disconnected = false;
    bool step() { return ++calls != fail_at && !disconnected; }
    bool read(unsigned, std::uint16_t a, std::uint32_t& v) override {
        if (!step()) return false;
        v = mmio[a]; return true;
    }
    bool write(unsigned width, std::uint16_t a, std::uint32_t v) override {
        if (!step()) return false;
        writes.emplace_back(width, a, v); mmio[a] = v; return true;
    }
    bool read_lte(std::uint16_t a, std::uint32_t& v) override {
        if (!step()) return false;
        v = lte[a]; return true;
    }
    bool write_lte(std::uint16_t a, std::uint32_t v) override {
        if (!step()) return false;
        writes.emplace_back(0, a, v); lte[a] = v; return true;
    }
};
void descriptors() {
    std::vector<std::uint8_t> frame(24 + 4096, 0x5a);
    // USB-style body: STF set, 63 old address entries, no optional WD info.
    std::fill_n(frame.begin(), 24, std::uint8_t{0});
    frame[1] = 4; frame[7] = 0xfc;
    std::array<std::uint8_t, 128> page{};
    std::array<std::uint8_t, 4096> payload{};
    check(be::make_wd_page(frame, 511, 0x12345000, page, payload) == 40, "B WD size");
    // Literal wire vector, derived from rtw89 pci.h old addr_info layout.
    const std::array<std::uint8_t, 16> expected{
        0xff,0x81,0,0,0,0,0,0, 0,0x10,1,0x80,0,0x50,0x34,0x12};
    check(std::equal(expected.begin(), expected.end(), page.begin() + 24), "B wire vector (no CE fragmentation)");
    check(page[0] == 0x80 && page[1] == 0 && page[7] == 4, "WD_PAGE, STF, address count");
    check(payload.front() == 0x5a && payload.back() == 0x5a, "entire payload copied");
    check(std::all_of(page.begin() + 40, page.end(), [](auto b) { return b == 0; }), "stale page data cleared");

    frame.resize(48 + 1); frame[2] = 0x40; frame.back() = 0x42;
    check(be::make_wd_page(frame, 0, 0xffffffff, page, payload) == 64, "optional WD info and last DMA byte");
    check(page[48] == 0 && page[49] == 0x80 && page[56] == 1 && payload[0] == 0x42, "WD info offsets");
    const auto saved_page = page;
    const auto saved_payload = payload;
    frame.push_back(0x43);
    check(be::make_wd_page(frame, 0, 0xffffffff, page, payload) == 0, "DMA end crosses 4GiB");
    check(be::make_wd_page(frame, 0, 0x100000000ull, page, payload) == 0, "high DMA start rejected");
    check(be::make_wd_page(frame, 0, ~std::uint64_t{0}, page, payload) == 0, "DMA wrap rejected");
    check(be::make_wd_page(frame, 512, 0, page, payload) == 0, "invalid release sequence");
    check(be::make_wd_page(frame, 0, 0, std::span(page).first(63), payload) == 0, "short WD page");
    check(be::make_wd_page(frame, 0, 0, page, std::span(payload).first(1)) == 0, "short payload bounce");
    check(be::make_wd_page(std::span(frame).first(47), 0, 0, page, payload) == 0, "truncated optional info");
    check(be::make_wd_page(std::span(frame).first(48), 0, 0, page, payload) == 0, "empty payload");
    check(page == saved_page && payload == saved_payload, "failed encoding has no side effects");
    frame.resize(48 + 4097);
    check(be::make_wd_page(frame, 0, 0, page, payload) == 0, "oversized packet");
}
void channels() {
    unsigned active = 0, bdram_end = 0;
    for (unsigned ch = 0; ch < 256; ++ch) {
        const bool expected = ch <= 3 || ch == 8 || ch == 9 || ch == 12;
        check(be::active_channel(ch) == expected, "unsupported TX channels rejected without reindexing");
        if (!expected) continue;
        const auto r = be::tx_rings[ch];
        check(r.start == bdram_end, "single BDRAM non-overlapping contiguous budget");
        check(r.base >= 0x1110 && r.base <= 0x1160, "B DESA bank (not CE)");
        bdram_end += r.maximum; ++active;
    }
    check(active == 7 && bdram_end == 32, "B BDRAM capacity");
    check(be::tx_rings[8].bdram == 0x1220 && be::tx_rings[12].base == 0x1160, "management/FWCMD golden register map");
    check(!be::valid_dma(0, 0) && be::valid_dma(0, 1ull << 32), "DMA range boundaries");
}
void identity() {
    FakeRegisters io;
    io.mmio[0x00f0] = 0x0000b0d4; io.mmio[0x00fc] = 0x12345651; io.mmio[0x03f0] = 0x100;
    be::Identity id;
    check(be::read_identity(io, id), "identity from always-on registers");
    check(id.rtl8852b() && id.die() == 0x51 && id.cut() == 0xb && id.mac_power() == 1,
          "die id byte, SYS_CFG1 cut and MAC power state");
    check(io.writes.empty() && io.calls == 7, "identity only reads");
    io.mmio[0x00fc] = 0x52;
    check(be::read_identity(io, id) && !id.rtl8852b(), "8852C die is not taken for 8852B");
    io.mmio[0x00f4] = 0xeaeaeaea;
    check(!be::read_identity(io, id), "unpowered domain is not an identity");
    for (unsigned failure = 1; failure <= 7; ++failure) {
        FakeRegisters broken; broken.fail_at = failure;
        check(!be::read_identity(broken, id) && broken.writes.empty(), "failed read fails identity");
    }
}
void coexistence() {
    FakeRegisters io;
    io.mmio[0x40] = 0xc1; io.mmio[0x41] = 0x84;
    io.mmio[0xc340] = 0x8800 | 0x200;
    io.mmio[0x73] = 0xa0;
    io.lte[0x38] = 0x77007700; io.lte[0x3c] = 0x1f5;
    check(be::initialize_hardware_pta(io), "PTA init");
    check(io.lte[0x38] == 0 && io.lte[0x3c] == 0x100, "old WiFi-only grants released");
    check(io.mmio[0x40] == 0x21 && io.mmio[0x41] == 0x82, "RTK/internal PTA preserves unrelated bits");
    check(io.mmio[0xc340] == 0x8820 && io.mmio[0x73] == 0xa4, "BTCCA restored and WL owns arbitration");
    check(std::count_if(io.writes.begin(), io.writes.end(), [](auto w) {
        return std::get<1>(w) == 0xc340 && std::get<0>(w) == 2;
    }) == 1, "CCA word width");
    const auto init_calls = io.calls;
    check(be::set_pta_band(io, true), "5GHz policy");
    check(io.lte[0x38] == 0x33003300 && io.lte[0x3c] == 0x1f0, "5GHz keeps BT under hardware control");
    check(be::set_pta_band(io, false) && io.lte[0x38] == 0 && io.lte[0x3c] == 0x100, "2.4GHz restores both hardware grants");
    check(be::release_pta_to_bluetooth(io) && io.mmio[0x73] == 0xa0, "shutdown returns PTA owner, preserves other bits");
    for (unsigned failure = 1; failure <= init_calls; ++failure) {
        FakeRegisters broken;
        broken.mmio[0x73] = 0xa4; broken.fail_at = failure;
        check(!be::initialize_hardware_pta(broken), "any register/LTE failure fails init");
        check(broken.mmio[0x73] == 0xa0, "failed init attempts to give BT ownership back");
    }
    for (unsigned failure = 1; failure <= 6; ++failure) {
        FakeRegisters broken;
        broken.lte[0x38] = 0x33003300; broken.lte[0x3c] = 0x1f0;
        broken.mmio[0xc67c] = 0x100; broken.fail_at = failure;
        check(!be::set_pta_band(broken, true), "band policy failure is propagated");
        check(broken.lte[0x38] == 0x33003300 && broken.lte[0x3c] == 0x1f0 && broken.mmio[0xc67c] == 0x100,
              "partial grant/PLT update rolls back to previous band");
    }
}
void coexistence_session() {
    FakeRegisters io;
    io.mmio[0x73] = 0xa0; io.lte[0x3c] = 0x100;
    be::PtaSession session;
    check(!session.transmit_allowed(), "no TX before PTA initialization");
    check(session.initialize(io) && session.transmit_allowed(), "active PTA allows TX");
    check(session.set_band(io, false) && io.mmio[0xc67c] == 0x166, "2.4GHz PHY0 priority masking");
    check(session.set_band(io, true) && io.mmio[0xc67c] == 0x100, "5GHz priority masking off");
    io.fail_at = io.calls + 5; // LTE0x38 succeeds, LTE0x3c fails.
    check(!session.set_band(io, false), "partially applied band change fails session");
    check(!session.transmit_allowed() && !session.cleanup_pending(), "successful cleanup does not unlatch failure");
    check(io.lte[0x38] == 0 && io.lte[0x3c] == 0x100 && io.mmio[0x73] == 0xa0 && io.mmio[0xc67c] == 0x100,
          "failed session leaves BT owner, neutral grants and WOFF PLT");
    const auto calls = io.calls;
    check(!session.set_band(io, true) && !session.initialize(io) && io.calls == calls, "faulted session cannot resume or touch radio");
    check(session.shutdown(io), "repeated shutdown idempotent");

    FakeRegisters unavailable;
    be::PtaSession lost;
    check(lost.initialize(unavailable), "initialize disconnect case");
    unavailable.disconnected = true;
    check(!lost.set_band(unavailable, true) && lost.cleanup_pending() && !lost.transmit_allowed(), "disconnect preserves pending cleanup and TX refusal");
    unavailable.disconnected = false;
    check(lost.shutdown(unavailable) && !lost.cleanup_pending() && !lost.transmit_allowed(), "reconnected bus retries cleanup without restarting TX");

    FakeRegisters boot_broken; boot_broken.disconnected = true;
    be::PtaSession boot;
    check(!boot.initialize(boot_broken) && boot.cleanup_pending(), "failed initialization retains cleanup even without active PTA");
    boot_broken.disconnected = false;
    check(boot.shutdown(boot_broken) && !boot.transmit_allowed(), "failed initial session cleanup retry");
}
}
int main() {
    try {
        descriptors(); channels(); identity(); coexistence(); coexistence_session();
        std::cout << checks << " BE descriptor, DMA boundary, identity and PTA failure checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAILED after " << checks << " checks: " << error.what() << '\n';
        return 1;
    }
}
