#include "power.h"

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
struct Xtal {
    std::uint8_t offset, value, mask;
    bool operator==(const Xtal&) const = default;
};

// Byte-addressed register file with the handshakes the sequences wait on:
// ONMAC/OFFMAC self-clear and move the power state, XTAL SI and efuse
// commands complete at once. Starts from the TX20E's registers at boot.
struct Chip final : be::MmioIo {
    std::map<std::uint16_t, std::uint8_t> bytes;
    std::vector<std::tuple<unsigned, std::uint16_t, std::uint32_t>> writes;
    std::vector<Xtal> xtal;
    std::vector<std::uint8_t> efuse = std::vector<std::uint8_t>(0x600, 0xff);
    unsigned calls = 0, fail_at = 0, waited = 0;
    bool disconnected = false, no_syspwr = false, xtal_stuck = false;
    Chip() {
        put(4, 0x0000, 0x51dcfee1); put(4, 0x0004, 0x50020082); put(4, 0x0088, 0x00000404);
        put(4, 0x00f0, 0x0c491d39); put(4, 0x03f0, 0x02200005);
        put(4, 0x0008, 0x00200000); // SYS_WL_EFUSE_CTRL (0x000a) autoload
        put(4, 0x0070, 1u << 12); put(4, 0x02f0, 1u << 17);
    }
    std::uint32_t get(unsigned width, std::uint16_t a) const {
        std::uint32_t v = 0;
        for (unsigned i = 0; i < width; ++i) {
            const auto it = bytes.find(static_cast<std::uint16_t>(a + i));
            v |= std::uint32_t{it == bytes.end() ? std::uint8_t{0} : it->second} << (8 * i);
        }
        return v;
    }
    void put(unsigned width, std::uint16_t a, std::uint32_t v) {
        for (unsigned i = 0; i < width; ++i)
            bytes[static_cast<std::uint16_t>(a + i)] = static_cast<std::uint8_t>(v >> (8 * i));
    }
    bool step() { return ++calls != fail_at && !disconnected; }
    bool read(unsigned width, std::uint16_t a, std::uint32_t& v) override {
        if (!step()) return false;
        if (no_syspwr) put(4, 0x0004, get(4, 0x0004) & ~(1u << 17));
        v = get(width, a);
        return true;
    }
    bool write(unsigned width, std::uint16_t a, std::uint32_t v) override {
        if (!step()) return false;
        writes.emplace_back(width, a, v);
        put(width, a, v);
        auto pw = get(4, 0x0004);
        if (pw & (1u << 8)) { pw &= ~(1u << 8); put(4, 0x03f0, (get(4, 0x03f0) & ~0x300u) | 0x100); }
        if (pw & (1u << 9)) { pw &= ~(1u << 9); put(4, 0x03f0, get(4, 0x03f0) & ~0x300u); }
        put(4, 0x0004, pw);
        if (a == 0x0270 && (v >> 31) && !xtal_stuck) {
            xtal.push_back({static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8),
                            static_cast<std::uint8_t>(v >> 16)});
            put(4, 0x0270, v & ~(1u << 31));
        }
        if (a == 0x0030) {
            const auto address = (v >> 16) & 0x7ff;
            put(4, 0x0030, (1u << 29) | (address << 16) | efuse[address]);
        }
        return true;
    }
    void delay_us(unsigned us) override { waited += us; }
    unsigned count(std::uint16_t a) const {
        return static_cast<unsigned>(std::count_if(writes.begin(), writes.end(),
            [a](auto w) { return std::get<1>(w) == a; }));
    }
    unsigned mac_state() const { return (get(4, 0x03f0) >> 8) & 3; }
};

const std::vector<Xtal> on_xtal{
    {0x90, 0x40, 0x40}, {0x90, 0x20, 0x20}, {0x90, 0x04, 0x04}, {0x90, 0x08, 0x08},
    {0x90, 0x00, 0x10}, {0x90, 0x01, 0x01}, {0x90, 0x02, 0x02}, {0x90, 0x00, 0x80},
    {0xa1, 0x00, 0x02}, {0x24, 0x00, 0x70}, {0x26, 0x00, 0x0f}};
const std::vector<Xtal> off_xtal{
    {0x90, 0x10, 0x10}, {0x90, 0x00, 0x08}, {0x90, 0x00, 0x04}, {0x80, 0x00, 0x01},
    {0x81, 0x00, 0x01}, {0x90, 0x80, 0x80}, {0x90, 0x00, 0x02}, {0x90, 0x00, 0x01},
    {0x90, 0x00, 0x20}, {0x90, 0x00, 0x40}};

void power_on() {
    Chip chip;
    const auto result = be::power_on(chip, {});
    check(static_cast<bool>(result) && result.xtal_mismatches == 0, "first power-on from boot state");
    const auto pw = chip.get(4, 0x0004);
    check(chip.get(2, 0x0228) == 0x4f42, "SPS analog default");
    check(!(pw & 0x1800) && (pw & (1u << 18)) && !(pw & (1u << 15)) && !(pw & (1u << 10)) &&
          (pw & (1u << 16)), "SYS_PW_CTRL suspend, LPS and WLON bits");
    check(chip.mac_state() == 1, "ONMAC handshake leaves the MAC on");
    check(chip.get(4, 0x0090) & 2, "WLBT LPS disabled");
    check((chip.get(4, 0x0400) & 0xff) == 0x31, "unknown RFE: C1=1, C3=3, C2/R1 untouched");
    check(chip.count(0x0088) == 5 && (chip.get(1, 0x0088) & 1), "PLATFORM_EN toggled twice, left on");
    check(!(chip.get(4, 0x0070) & (1u << 12)), "PCIe calibration enable cleared");
    check((chip.get(4, 0x0018) & 0x60) == 0x60, "WL PTA and RFC pads up");
    check(chip.xtal == on_xtal, "XTAL SI on-sequence");
    const auto iso = chip.get(4, 0x0000);
    check((iso & (1u << 8)) && !(iso & 0xc000) && !(chip.get(4, 0x00cc) & 4),
          "isolation released, PMC write mask closed");
    check(chip.count(0x0200) == 0 && chip.count(0x007a) == 0, "no voltage trim before the efuse is read");
    check(chip.get(4, 0x8400) == 0x7fff8000 && chip.get(4, 0xc000) == 0x7000803f, "DMAC and CMAC enabled");
    check(((chip.get(4, 0x02d8) >> 4) & 0xf) == 1, "EESK pin to BT log");
    check(chip.get(1, 0x00af) == 0x81, "scoreboard tells BT the WL is up");
    check(chip.waited >= 1000, "isolation settle delay");
}

void voltage_trim() {
    Chip cbv;
    check(static_cast<bool>(be::power_on(cbv, {1, true, false, 0})), "CBV power-on with efuse, no power-K");
    const auto on = cbv.get(4, 0x0200);
    check((on & 0xf) == 9 && ((on >> 22) & 0xf) == 0xa, "SPS VOL_L1 and VREFPFM_L trim");
    check((cbv.get(2, 0x007a) & 0xf) == 0xa, "CBV PCIe HCI LDO trim");
    const auto ldo = std::find_if(cbv.writes.begin(), cbv.writes.end(),
                                  [](auto w) { return std::get<1>(w) == 0x007a; });
    check(ldo != cbv.writes.begin() && std::get<1>(*(ldo - 1)) == 0x00cc && (std::get<2>(*(ldo - 1)) & 4) &&
          std::get<1>(*(ldo + 1)) == 0x00cc && !(std::get<2>(*(ldo + 1)) & 4), "HCI LDO written inside the PMC write mask");
    Chip cav;
    check(be::power_on(cav, {0, true, false, 0}) && cav.count(0x0200) == 2 && cav.count(0x007a) == 0,
          "A-cut gets the SPS trim only");
    Chip calibrated;
    check(be::power_on(calibrated, {1, true, true, 0}) && calibrated.count(0x0200) == 0 &&
          calibrated.count(0x007a) == 0, "valid power-K skips the trim");
    Chip rfe5;
    check(be::power_on(rfe5, {1, false, false, 5}) && (rfe5.get(4, 0x0400) & 0xff) == 0x65,
          "RFE 5: C1=1, C2=1, C3=2, R1=1");
}

void power_on_failures() {
    Chip on;
    on.put(4, 0x03f0, 0x02200105);
    const auto busy = be::power_on(on, {});
    check(busy.error == be::PowerError::mac_on && busy.address == 0x03f0 && on.writes.empty(),
          "MAC already on is refused before any write");
    Chip dead;
    dead.no_syspwr = true;
    const auto syspwr = be::power_on(dead, {});
    check(syspwr.error == be::PowerError::timeout && syspwr.address == 0x0004 && dead.waited == 20000,
          "RDY_SYSPWR polled 20ms");
    Chip stuck;
    stuck.xtal_stuck = true;
    const auto xtal = be::power_on(stuck, {});
    check(xtal.error == be::PowerError::xtal_timeout && xtal.address == 0x90 && stuck.waited == 50000,
          "XTAL SI timeout names the offset");
    Chip good;
    be::power_on(good, {});
    for (unsigned failure = 1; failure <= good.calls; ++failure) {
        Chip broken;
        broken.fail_at = failure;
        const auto result = be::power_on(broken, {});
        check(!result && (result.error == be::PowerError::read || result.error == be::PowerError::write),
              "every I/O failure fails power-on");
    }
    Chip lost;
    lost.put(4, 0x0400, 0xeaeaeaea);
    const auto unpowered = be::power_on(lost, {});
    check(unpowered.error == be::PowerError::read && unpowered.address == 0x0400, "lost dword is a read failure");
}

void power_off() {
    Chip chip;
    check(static_cast<bool>(be::power_on(chip, {})), "power-on before off");
    chip.xtal.clear();
    chip.put(1, 0x0002, 0xff);
    const auto result = be::power_off(chip);
    check(static_cast<bool>(result), "power-off");
    check(chip.mac_state() == 0, "OFFMAC handshake leaves the MAC off");
    check(chip.xtal == off_xtal, "XTAL SI off-sequence");
    check(chip.get(4, 0x0090) == 0x0001a0b2, "PCIe SW LPS option");
    const auto pw = chip.get(4, 0x0004);
    check((pw & (1u << 10)) && (pw & (1u << 16)), "PCIe ends in SW LPS");
    check((chip.get(4, 0x0010) & (1u << 10)) && ((chip.get(4, 0x0200) >> 17) & 3) == 3, "SPS PWM and ZCDC");
    check(!(chip.get(4, 0x02f0) & (1u << 17)) && (chip.get(1, 0x0002) & 3) == 0, "AFE digital and BB reset");
    check((chip.get(4, 0x0018) & 0x60) == 0, "WL pads down");
    check(chip.get(1, 0x00af) == 0x80, "scoreboard tells BT the WL is down");
    for (unsigned failure = 1; failure <= 40; ++failure) {
        Chip broken;
        be::power_on(broken, {});
        broken.fail_at = broken.calls + failure;
        const auto off = be::power_off(broken);
        check(!off && broken.get(1, 0x00af) == 0x81, "failed power-off does not notify BT");
    }
}

void efuse() {
    Chip chip;
    // RFE and country (block 0x59 word 1), XTAL (block 0x57 word 0), PCIe MAC
    // (block 0x80 words 0-2), then the end marker.
    const std::vector<std::uint8_t> blocks{
        0x35, 0x9d, 0x01, 0x4b,
        0x35, 0x7e, 0x00, 0x3f,
        0x38, 0x08, 0x00, 0xe0, 0x4c, 0x12, 0x34, 0x56};
    std::copy(blocks.begin(), blocks.end(), chip.efuse.begin() + 4);
    chip.efuse[0x5e9] = 0xaa;
    check(static_cast<bool>(be::power_on(chip, {})), "power-on before efuse");
    const auto before = chip.count(0x0030);
    be::Efuse e;
    const auto result = be::read_efuse(chip, 1, e);
    check(static_cast<bool>(result) && e.autoload, "efuse read with autoload");
    check(chip.count(0x0030) - before == be::physical_efuse_bytes + be::phycap_bytes, "one command per byte");
    check(e.rfe() == 0x01 && e.logical[0x2cb] == 0x4b && e.xtal() == 0x3f, "RFE, country and XTAL");
    const std::array<std::uint8_t, 6> mac{0x00, 0xe0, 0x4c, 0x12, 0x34, 0x56};
    check(std::equal(mac.begin(), mac.end(), e.mac().begin()), "PCIe MAC at 0x400");
    check(e.logical[0] == 0xff && e.logical[0x406] == 0xff, "unprogrammed logical bytes stay 0xff");
    check(e.power_k() && e.external_pn() == 0xff, "phycap power-K and security bytes");
    const auto iso = chip.get(4, 0x0000);
    check((iso & (1u << 8)) && !(iso & 0xc000) && !(chip.get(1, 0x00cc) & 4), "efuse power cut restored");
    check(chip.count(0x0038) == 0, "burst mode is A-cut only");

    Chip cav;
    check(be::power_on(cav, {0, false, false, 0}) && be::read_efuse(cav, 0, e) &&
          cav.count(0x0038) == 4 && !(cav.get(4, 0x0038) & (1u << 19)), "A-cut burst on and off per dump");

    Chip flaky;
    be::power_on(flaky, {});
    flaky.fail_at = flaky.calls + 40;
    check(static_cast<bool>(be::read_efuse(flaky, 1, e)), "efuse dump retried after one failure");
    Chip gone;
    be::power_on(gone, {});
    gone.disconnected = true;
    check(be::read_efuse(gone, 1, e).error == be::PowerError::read, "lost device fails the efuse read");
}

void decoding() {
    std::vector<std::uint8_t> physical(be::physical_efuse_bytes, 0xff);
    std::array<std::uint8_t, be::logical_efuse_bytes> logical;
    logical.fill(0xff);
    check(be::decode_efuse(physical, logical) &&
          std::all_of(logical.begin(), logical.end(), [](auto b) { return b == 0xff; }), "blank efuse");
    physical[4] = 0x3f; physical[5] = 0xf0; // block 0xff, all four words
    for (unsigned i = 0; i < 8; ++i) physical[6 + i] = static_cast<std::uint8_t>(i);
    check(be::decode_efuse(physical, logical) && logical[0x7f8] == 0 && logical[0x7ff] == 7, "last logical block");
    physical[14] = 0x30; physical[15] = 0x0e;
    physical[16] = 0xaa; physical[17] = 0xbb;
    check(be::decode_efuse(physical, logical) && logical[0] == 0xaa, "blocks follow each other");
    // One two-word block, then one-word blocks until a header lands two bytes
    // before the end, leaving no room for its word.
    std::vector<std::uint8_t> truncated(be::physical_efuse_bytes, 0x00);
    const auto end = be::physical_efuse_bytes - 4;
    std::size_t at = 4;
    truncated[at] = 0x30; truncated[at + 1] = 0x0c; at += 6;
    for (; at <= end - 2; at += 4) { truncated[at] = 0x30; truncated[at + 1] = 0x0e; }
    check(!be::decode_efuse(truncated, logical), "word past the physical end is rejected");
}
}

int main() {
    try {
        power_on(); voltage_trim(); power_on_failures(); power_off(); efuse(); decoding();
        std::cout << checks << " BE power and efuse checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAILED after " << checks << " checks: " << error.what() << '\n';
        return 1;
    }
}
