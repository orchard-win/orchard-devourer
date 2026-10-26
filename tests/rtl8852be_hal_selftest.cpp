// Executes the actual devourer HAL against a byte-addressed transport.
// This proves register selection/values and failure propagation, not hardware.
#include "kestrel/HalKestrel.h"
#include "kestrel/KestrelFw.h"
#include "kestrel/RtlKestrelDevice.h"
#include "kestrel/FrameParserKestrel.h"
#include "kestrel/TxDescKestrel.h"
#include "rtl8852be.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
class MemoryTransport final : public devourer::ITransport {
public:
    bool usb = false, lte_broken = false, host_ltr = false;
    bool allow_tx = false, tx_fails = false, rings_fail = false;
    unsigned setups = 0;
    std::optional<std::uint16_t> fail_write_address;
    std::array<std::uint8_t, 65536> bytes{};
    std::array<std::uint8_t, 4096> config{};
    std::array<std::array<std::uint16_t, 64>, 2> phy{};
    std::vector<std::uint8_t> sent;
    std::map<std::uint16_t, std::uint32_t> lte;
    // With `chip`, the handshakes power-on and firmware download wait on:
    // ONMAC/OFFMAC move the power state (OFFMAC also resets the WCPU), XTAL SI
    // and efuse reads complete, the boot ROM opens the H2C path once the WCPU
    // runs, the header opens the download path and a section brings the
    // firmware up.
    bool chip = false;
    std::vector<std::vector<std::uint8_t>> packets;
    std::uint32_t stops_at_download = 0;
    std::map<std::uint32_t, std::uint32_t> wide;
    std::array<std::uint8_t, 2048> efuse; // physical, blank unless a test writes it
    std::vector<bool> efuse_read = std::vector<bool>(2048);
    std::vector<std::uint16_t> written; // every byte address, in order
    std::vector<devourer::DmaState> dma_states; // as the HAL reported them
    unsigned drains = 0;
    MemoryTransport() { config[0x82] = 2; config[0x719] = 8; efuse.fill(0xff); }
    void seed(std::uint16_t a, std::uint32_t v) {
        for (unsigned i = 0; i < 4; ++i) bytes[a + i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
    void boot() { // the TX20E's registers at boot
        chip = true;
        seed(0x0000, 0x51dcfee1); seed(0x0004, 0x50020082); seed(0x0088, 0x00000404);
        seed(0x00f0, 0x0c491d39); seed(0x03f0, 0x02200005); seed(0x0008, 0x00200000);
        seed(0x0070, 1u << 12); seed(0x02f0, 1u << 17);
    }
    unsigned mac_state() const { return bytes[0x03f1] & 3; }
    bool is_usb() const override { return usb; }
    bool ltr_allowed() override { return host_ltr; }
    void set_dma_state(devourer::DmaState state) override { dma_states.push_back(state); }
    void hci_setup() override {
        ++setups;
        if (rings_fail) throw std::runtime_error("injected ring failure");
    }
    devourer::UsbLinkInfo usb_info() const override {
        devourer::UsbLinkInfo info;
        if (usb) {
            info.valid = true; info.speed = devourer::kUsbSpeedSuper;
            info.bulk_out_eps = {0x02, 0x03, 0x07};
        }
        return info;
    }
    std::uint8_t read8(std::uint16_t a) override {
        // A lost LTE engine reads as an all-ones dword, not one 0xff byte.
        if (a >= 0xdaf0 && a <= 0xdaf3 && lte_broken) return 0xff;
        if (a == 0xdaf3) return chip && mac_state() != 1 ? 0 : 0x20; // ready while the MAC is on
        return bytes[a];
    }
    std::uint16_t read16(std::uint16_t a) override {
        auto v = std::uint16_t(read8(a) | (std::uint16_t{read8(a + 1)} << 8));
        if (a == 0xce34) v &= ~std::uint16_t{0x100}; // CAM reset completed
        return v;
    }
    std::uint32_t read32(std::uint16_t a) override {
        if (a == 0x8d00 || a == 0x9100) return 3; // DLE init ready
        if (a == 0x9e10) return 0x80000000; // scheduler init ready
        return read16(a) | (std::uint32_t{read16(a + 2)} << 16);
    }
    bool write8(std::uint16_t a, std::uint8_t v) override {
        if (a == fail_write_address) return false;
        written.push_back(a);
        bytes[a] = v;
        if (chip && a >= 0x0004 && a <= 0x0007) {
            if (bytes[0x0005] & 1) { bytes[0x0005] &= ~1; bytes[0x03f1] = (bytes[0x03f1] & ~3) | 1; }
            if (bytes[0x0005] & 2) { bytes[0x0005] &= ~2; bytes[0x03f1] &= ~3; seed(0x01e0, 0); }
        }
        if (chip && a == 0x0273) bytes[a] &= 0x7f;            // XTAL SI done
        if (chip && a == 0x0088 && (v & 2)) bytes[0x01e0] |= 2; // H2C path ready
        if (a == 0x1092) {
            const auto ctl = read16(0x1090);
            const auto base = ctl & 0xffc;
            for (unsigned i = 0; i < 4; ++i) {
                if (v == 2) bytes[0x1098+i] = config[base+i];
                if (v == 1 && (ctl & (1 << (12+i)))) config[base+i] = bytes[0x1094+i];
            }
            bytes[a] = 0;
        }
        return true;
    }
    bool write16(std::uint16_t a, std::uint16_t v) override {
        if (a == fail_write_address) return false;
        write8(a, static_cast<std::uint8_t>(v)); write8(a + 1, static_cast<std::uint8_t>(v >> 8));
        if (a == 0x10a0 && (v & 0x300)) {
            auto& reg = phy[(v >> 13) & 1][(v & 0x1f) + ((v & 0x1000) ? 32 : 0)];
            if (v & 0x100) reg = read16(0x10a4);
            if (v & 0x200) write16(0x10a6, reg);
            bytes[0x10a1] &= ~3;
        }
        return true;
    }
    bool write32(std::uint16_t a, std::uint32_t v) override {
        if (a == fail_write_address) return false;
        if (a == 0x1000) v &= ~8u;
        write16(a, static_cast<std::uint16_t>(v)); write16(a + 2, static_cast<std::uint16_t>(v >> 16));
        if (a == 0xdaf0) {
            const auto offset = static_cast<std::uint16_t>(v);
            if ((v & 0xc0000000) == 0xc0000000) lte[offset] = read32(0xdaf4);
            else write32(0xdaf8, lte[offset]);
        }
        if (chip && a == 0x0030) {
            const auto at = (v >> 16) & 0x7ff;
            efuse_read[at] = true;
            seed(a, (1u << 29) | (v & 0x07ff0000) | efuse[at]);
        }
        return true;
    }
    bool write32_wide(std::uint32_t a, std::uint32_t v) override {
        if (a < 0x10000) return write32(static_cast<std::uint16_t>(a), v);
        wide[a] = v;
        return true;
    }
    bool write_bytes(std::uint16_t a, const std::uint8_t* p, std::size_t n) override {
        for (std::size_t i = 0; i < n; ++i) write8(static_cast<std::uint16_t>(a + i), p[i]);
        return true;
    }
    bool tx_async(std::uint8_t, std::uint8_t*, std::size_t, unsigned) override { throw std::runtime_error("unexpected TX"); }
    int tx_sync(std::uint8_t, std::uint8_t* p, std::size_t n, int) override {
        if (!allow_tx) throw std::runtime_error("unexpected TX");
        sent.assign(p, p+n);
        if (chip) {
            packets.emplace_back(p, p + n);
            if (packets.size() == 1) stops_at_download = read32(0x1010);
            bytes[0x01e0] |= (p[2] & 0x10) ? 0xe0 : 4; // section: firmware ready; header: download path
        }
        return tx_fails ? -1 : static_cast<int>(n);
    }
    // Only the stop's drain reads the ring here; the ring is empty.
    void rx_loop(int, int, const std::function<void(const std::uint8_t*, int)>&,
                 const std::function<bool()>&) override { ++drains; }
    devourer::DmaState dma_state() const { return dma_states.empty() ? devourer::DmaState::unknown : dma_states.back(); }
};

void quotas(Logger_t logger, bool usb, kestrel::ChipVariant chip) {
    const auto bus = std::make_shared<MemoryTransport>(); bus->usb = usb;
    RtlAdapter adapter(bus, logger);
    kestrel::KestrelFw fw(adapter, logger, chip);
    check(fw.fw_pre_init(), "firmware pre-init on fake bus");
    const bool c = chip == kestrel::ChipVariant::C8852C;
    check(bus->read32(0x8c44) == (c ? 0u : usb ? 0x00060006u : 0x00300030u), "DLFW WCPU quota per bus");
    const auto base = static_cast<std::uint16_t>(c ? 0x1700 : 0x8a00);
    check(bus->read32(base + 4) == ((usb ? 32u : c ? 256u : 40u) << 16), "DLFW precost golden value");
    check(((bus->read32(base) >> 10) & 3) == ((!usb && !c) ? 0u : 1u), "BE DLFW full condition X1");

    kestrel::HalKestrel hal(adapter, logger, chip);
    check(hal.trx_dmac_init(), "actual HAL DMAC init");
    if (!usb && !c) {
        check(bus->read32(0x8c40) == 0x01be01be, "BE WDE host 446/446");
        check(bus->read32(0x8c44) == 0x00300030, "BE WCPU 48/48");
        check(bus->read32(0x8c50) == 0x00100010, "BE CPU I/O 16/16");
        check((bus->read32(0x8c08) >> 16) == 510, "BE WDE linked 510");
        check((bus->read32(0x9008) >> 16) == 496, "BE PLE linked 496");
        check(bus->read32(0x8a10) == 0x01550005, "BE CH0 5/341");
        check(bus->read32(0x8a18) == 0x01560004, "BE CH2 4/342");
        check(bus->read32(0x8a90) == 446 && bus->read32(0x8a94) == 446, "BE public budget");
        check(bus->read32(0x8a04) == 0x00280002, "BE runtime precost");
        check((bus->read32(0x8a00) & 0xffe) == 0x18, "BE POH and full conditions");
        check(bus->read32(0x9058) == 0x00e50059, "BE RX PLE min89 max229");
        check(bus->read32(0x9068) == 0x00180008, "BE PLE Q10 min8 max24");
        check(bus->read32(0x906c) == 0, "BE PLE Q11 zero");
    } else if (usb && !c) {
        check(bus->read32(0x8c40) == 0x00d200d2 && bus->read32(0x8a90) == 210, "USB B characterized quota preserved");
        check((bus->read32(0x8a00) & 6) == 2, "USB remains STF");
        check(!hal.power_off(), "power-off is BE PCIe only");
    } else if (!usb && c) {
        check(bus->read32(0x8c40) == 0x0c9c0c9c, "CE host quota preserved");
        check(bus->read32(0x1790) == 0x064e064e, "CE dual group budget preserved");
    }
}

void pta(Logger_t logger, bool broken) {
    const auto bus = std::make_shared<MemoryTransport>();
    bus->lte_broken = broken; bus->write8(0x73, 0xa4);
    bus->write8(0xda20, 0xff); // a real 0xff byte must not read as a lost device
    bus->lte[0x38] = 0x77007700; bus->lte[0x3c] = 0x1f5;
    {
        kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
        hal.set_hardware_pta(true);
        check(hal.trx_cmac_rx_init() == !broken, "HAL must propagate LTE failure");
        if (!broken) {
            check(bus->lte[0x38] == 0 && bus->lte[0x3c] == 0x100, "HAL invokes PTA policy");
            check((bus->read16(0xc340) & 0x220) == 0x20, "PTA restores BTCCA after CMAC scheduler");
            check(bus->read8(0x73) == 0xa4, "WL owns active PTA");
            check(hal.coexistence_tx_allowed(), "active HAL allows TX");
            bus->lte_broken = true;
            hal.fail_coexistence();
            check(!hal.coexistence_tx_allowed(), "HAL fault latches TX refusal");
            bus->lte_broken = false; // destructor must retry incomplete cleanup
        }
    }
    check(bus->read8(0x73) == 0xa0, "HAL destructor/failure restores BT owner");
    if (!broken) check(bus->read16(0xc67c) == 0x100 && bus->lte[0x38] == 0, "HAL destructor retries WOFF cleanup");
}
void pcie_lifecycle(Logger_t logger) {
    for (const bool allowed : {false, true}) {
        const auto bus = std::make_shared<MemoryTransport>(); bus->host_ltr = allowed;
        kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
        check(!hal.pcie_init(), "HAL refuses post-init without successful preparation");
        check(hal.pcie_pre_init() && bus->setups == 1, "HAL dispatches BE PHY and ring preparation");
        check(bus->dma_state() == devourer::DmaState::running, "transport hears DMA running from preparation");
        check(bus->read32(0x1010) == 0xb0f00 && bus->config[0x719] == 8, "HAL BE FWDL-only channels and restored PCI config");
        check(hal.intf_init() && bus->read32(0x1010) == 0, "HAL post-init opens BE TX");
        check(bus->read32(0x8410) == (allowed ? 0x2743u : 0u), "HAL forwards host LTR permission");
        check(hal.pcie_deinit() && !(bus->read32(0x1000) & 0x2800), "HAL BE stop propagates success");
        check(bus->dma_state() == devourer::DmaState::stopped, "transport hears the confirmed stop");
        check(!hal.pcie_init(), "HAL post-init stays closed after stop");
    }
    const auto bus = std::make_shared<MemoryTransport>();
    kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
    bus->rings_fail = true;
    check(!hal.pcie_pre_init() && !hal.pcie_init(), "transport ring exception becomes failed HAL preparation");
    check(bus->dma_state() == devourer::DmaState::stopped, "failed preparation whose cleanup stop passed reports stopped");
    bus->rings_fail = false;
    check(hal.pcie_pre_init(), "retry preparation after transport recovers");
    bus->fail_write_address = 0x8410;
    check(!hal.intf_init() && !(bus->read32(0x1000) & 0x2800), "LTR write failure reaches bring-up and stops DMA");
    check(bus->dma_state() == devourer::DmaState::unconfirmed, "failed start whose cleanup failed reports unconfirmed");
    check(!hal.pcie_deinit(), "HAL teardown cannot report success on failed write");
    check(bus->dma_state() == devourer::DmaState::unconfirmed, "failed teardown reports unconfirmed");
}
// HalKestrel::shutdown_be, the live path's stop: drain while DMA is up, stop,
// power off what this HAL powered on, and what the transport hears.
void stop_contract(Logger_t logger) {
    {
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot(); // MAC off, as the TX20E boots
        kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
        const auto shutdown = hal.shutdown_be(true);
        check(shutdown.stopped && shutdown.powered_off && bus->written.empty() &&
              bus->dma_state() == devourer::DmaState::stopped,
              "a MAC that is off and was never started here is reported stopped without a stop");
    }
    const auto bus = std::make_shared<MemoryTransport>();
    bus->boot(); bus->allow_tx = true;
    bus->bytes[0x03f1] = 1; // left on by someone else
    kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
    auto shutdown = hal.shutdown_be(true);
    check(shutdown.stopped && shutdown.powered_off && bus->drains == 0 && bus->mac_state() == 1,
          "nothing started here: no drain, and no power-off of a MAC this HAL never powered");
    check(bus->dma_state() == devourer::DmaState::stopped, "the stop is still reported");

    check(hal.power_on() && hal.download_firmware(hal.read_cut()), "power-on and download for the stop");
    shutdown = hal.shutdown_be(true);
    check(shutdown.stopped && shutdown.powered_off && bus->drains == 1 && bus->mac_state() == 0 &&
          bus->dma_state() == devourer::DmaState::stopped, "drain, confirmed stop, power-off");
    shutdown = hal.shutdown_be(true);
    check(shutdown.stopped && shutdown.powered_off && bus->drains == 1, "a second stop has nothing to drain or power off");

    check(hal.power_on() && hal.download_firmware(hal.read_cut()), "power-on and download again");
    check(hal.shutdown_be(false).stopped && bus->drains == 1, "no drain when the caller does not own the ring");

    check(hal.power_on() && hal.download_firmware(hal.read_cut()), "power-on and download for a stuck stop");
    bus->seed(0x101c, 1); // RXQ stays busy
    shutdown = hal.shutdown_be(true);
    check(!shutdown.stopped && bus->dma_state() == devourer::DmaState::unconfirmed,
          "a stop that never saw idle reports unconfirmed");
    check(shutdown.powered_off && bus->mac_state() == 0, "power-off runs even after an unconfirmed stop");
}
// Autoloaded, no power-K, RFE 5; the PCIe block holds the MAC and PCI ids.
void program_efuse(MemoryTransport& bus) {
    bus.bytes[0x000a] = 0x20;
    const std::uint8_t programmed[] = {
        0x35, 0x9d, 0x05, 0xff,                                     // block 0x59 word 1: 0x2ca
        0x38, 0x00, 0x02, 0x11, 0x22, 0x33, 0x44, 0x55, 0xec, 0x10, // block 0x80: 0x400-0x407
        0x38, 0x1e, 0x52, 0xb8};                                    // block 0x81 word 0: 0x408
    std::copy(std::begin(programmed), std::end(programmed), bus.efuse.begin() + 4);
}
// The runtime power-on's cut B voltage trim without power-K, and the RFE 5 branch.
bool efuse_power_settings(MemoryTransport& bus) {
    return (bus.read32(0x0200) & 0x0f) == 0x9 && ((bus.read32(0x0200) >> 22) & 0xf) == 0xa &&
           (bus.bytes[0x007a] & 0x0f) == 0xa && bus.bytes[0x0400] == 0x65;
}
// Linux's probe-time partial init through the HAL (the probe's `fw` stage),
// then rtw89_core_start's power-on with the efuse's settings and the second
// download (the `runtime` stage), then the rest of rtw89_mac_init (`trx`) and
// devourer's BB/RF init (`rx`), TX setup (`tx`) and station identity (`ack`).
void probe_and_start(Logger_t logger) {
    const auto bus = std::make_shared<MemoryTransport>();
    bus->boot(); bus->allow_tx = true;
    bus->bytes[0x03f1] = 1; // MAC left on: refused, powered off, retried
    program_efuse(*bus);
    kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
    check(hal.power_on() && bus->mac_state() == 1, "BE power-on retried after power-off");
    check((bus->read32(0x0200) & 0x03c0000f) == 0 && (bus->bytes[0x007a] & 0x0f) == 0 && bus->bytes[0x0400] == 0x31,
          "first power-on: no voltage trim, default RFE branch");
    check(hal.download_firmware(hal.read_cut()), "BE firmware download over CH12");
    check(bus->setups == 1 && bus->stops_at_download == 0xb0f00, "rings armed and only CH12 open for the download");
    check(bus->read32(0x8c44) == 0x00300030, "BE download WCPU quota 48");
    check(bus->wide.count(0x40000) == 1, "CPU clock patch through the indirect window, not MAC space");
    const auto word = [](const auto& p, std::size_t i) {
        return p[i] | (p[i + 1] << 8) | (p[i + 2] << 16) | (std::uint32_t{p[i + 3]} << 24);
    };
    const auto wd = [&](const std::vector<std::uint8_t>& p, bool section) {
        return p.size() > 24 && p.size() <= 24 + 2020 && ((word(p, 0) >> 16) & 0xf) == 12 &&
               ((word(p, 0) & (1u << 20)) != 0) == section && (word(p, 8) & 0xffff) == p.size() - 24;
    };
    check(bus->packets.size() > 1 && wd(bus->packets.front(), false), "header: 24-byte CH12 WD without FWDL_EN");
    check(std::all_of(bus->packets.begin() + 1, bus->packets.end(), [&](const auto& p) { return wd(p, true); }),
          "sections: CH12 WD with FWDL_EN, at most 2020 bytes each");
    kestrel::EfuseInfo info;
    std::array<std::uint8_t, 1536> raw{};
    check(hal.read_efuse(info, &raw), "BE efuse read with the firmware running");
    const auto& read = bus->efuse_read;
    check(std::all_of(read.begin(), read.begin() + 1216, [](bool b) { return b; }) &&
          std::none_of(read.begin() + 1216, read.begin() + 0x580, [](bool b) { return b; }) &&
          std::all_of(read.begin() + 0x580, read.begin() + 0x600, [](bool b) { return b; }),
          "BE efuse: the 1216-byte map and the phycap, nothing between");
    check(std::equal(raw.begin(), raw.begin() + 1216, bus->efuse.begin()) &&
          std::all_of(raw.begin() + 1216, raw.end(), [](std::uint8_t b) { return b == 0xff; }),
          "raw map: 1216 bytes as read, the rest 0xff");
    const std::array<std::uint8_t, 6> mac{0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
    check(info.mac == mac && info.autoload_ok && info.pci_vid == 0x10ec && info.pci_did == 0xb852 &&
          info.rfe_type == 5, "BE efuse decoded: MAC, PCI ids and RFE");
    check(!hal.power_off() && bus->mac_state() == 1, "power-off refused while DMA is up");
    check(hal.pcie_deinit() && !(bus->read32(0x1000) & 0x2800), "DMA stopped before power-off");
    check(hal.power_off() && bus->mac_state() == 0, "BE power-off");

    const auto probe_packets = bus->packets.size();
    check(hal.power_on() && bus->mac_state() == 1, "BE power-on with the efuse's settings");
    check(efuse_power_settings(*bus), "runtime power-on: cut B voltage trim without power-K, RFE 5 branch");
    check(hal.download_firmware(hal.read_cut()) && bus->setups == 2 && bus->packets.size() == 2 * probe_packets,
          "second download after the runtime power-on");

    bus->written.clear();
    check(hal.enable_bb_rf() && (bus->bytes[0x0002] & 3) == 3, "enable_bb_rf: BB out of reset");
    check(std::none_of(bus->written.begin(), bus->written.end(), [](std::uint16_t a) { return a >= 4 && a < 8; }),
          "enable_bb_rf writes the FEN byte alone, not SYS_PW_CTRL");
    hal.mac_sys_init();
    check(hal.trx_dmac_init() && hal.trx_cmac_rx_init(), "DMAC and CMAC init");
    hal.enable_ser_imr();
    hal.set_host_rpr();
    check((bus->read32(0x9408) & 3) == 0 && (bus->read32(0x9414) & 0xff) == 30,
          "host release report: POH with rtw89's aggregation count 30");
    check(hal.intf_init() && bus->read32(0x1010) == 0, "PCIe post-init opens the BE channels and releases IO");

    bus->written.clear();
    const auto mac_packets = bus->packets.size();
    check(hal.phy_bb_rf_init(info.rfe_type, hal.read_cut()) && (bus->bytes[0x0002] & 3) == 3,
          "BB/RF init: BB out of reset after the pulse");
    check(std::none_of(bus->written.begin(), bus->written.end(), [](std::uint16_t a) { return a >= 4 && a < 8; }),
          "BB/RF init writes the FEN byte alone, not SYS_PW_CTRL");
    check((bus->read32(0x8a00) & 1) && bus->packets.size() > mac_packets,
          "HCI flow control on, then the radio pages over CH12");

    // Stage 7: devourer's TX enable, then its frames as the BE WD pages they become.
    namespace r = kestrel::reg;
    hal.set_cca_on(false, true);
    hal.port_init();
    hal.sch_tx_en();
    const auto queues = r::B_AX_CTN_TXEN_BE_0 | r::B_AX_CTN_TXEN_MGQ;
    check((bus->read32(r::R_AX_PORT_CFG_P0) & r::B_AX_PORT_FUNC_EN_P0) &&
          (bus->read16(r::R_AX_CTN_TXEN) & queues) == queues && !(bus->read32(r::R_AX_CCA_CFG_0) & r::B_AX_CCA_ALL_EN),
          "TX enable: port, BE and management queues on, carrier sense off");
    const auto filters = r::B_WDRLS_FLTR_TXOK | r::B_WDRLS_FLTR_RTYLMT | r::B_WDRLS_FLTR_LIFTIM | r::B_WDRLS_FLTR_MACID;
    check((bus->read32(r::R_AX_RLSRPT0_CFG0) & filters) == filters, "a release report for every TX result");
    const std::uint8_t self[6]{0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee};
    const auto role_packets = bus->packets.size();
    check(hal.register_sta_role(0, 0, 0) && hal.add_self_sta(self, 0) && bus->packets.size() == role_packets + 3,
          "station role, address CAM and CMAC control table: three H2Cs over CH12");
    for (const bool data : {false, true}) {
        std::vector<std::uint8_t> frame(24, 0xff);
        frame[0] = data ? 0x48 : 0x40;
        frame[1] = frame[2] = frame[3] = 0;
        std::copy_n(self, 6, frame.begin() + 10);
        const kestrel::TxRate rate;
        const auto wd = data ? kestrel::build_data_txdesc(frame.data(), 24, rate, 0, 5, kestrel::WD_BODY_LEN, 1, false)
                             : kestrel::build_mgnt_txdesc(frame.data(), 24, rate, 0, 5, kestrel::WD_BODY_LEN, 1, false);
        std::array<std::uint8_t, 128> page{};
        std::array<std::uint8_t, 4096> payload{};
        check(orchard::rtl8852be::make_wd_page(wd, 7, 0x00600000, page, payload) == 64 &&
              std::equal(frame.begin(), frame.end(), payload.begin()), "stage-7 frame: a 64-byte WD page and its payload");
        const auto body0 = word(page, 0), body2 = word(page, 8), info1 = word(page, 28);
        check((body0 & 0x80) && !(body0 & 0x400) && ((body0 >> 16) & 0xf) == (data ? 0u : 8u) &&
              (word(page, 4) >> 26) == 1 && (body2 & 0x3fff) == 24 && ((body2 >> 17) & 0x3f) == (data ? 0u : 18u),
              "stage-7 frame: WD page on ACH0 or B0MG, one address entry, size and queue kept");
        check((info1 & (1u << 11)) && (info1 >> 31) && ((info1 >> 25) & 0x3f) == 1,
              "stage-7 frame: group-addressed, one attempt");
        check(word(page, 48) == 0x8007 && word(page, 56) == (24u | 0x8001u << 16) && word(page, 60) == 0x00600000,
              "stage-7 frame: release sequence 7 and the payload address");
    }

    // Stage 8: a probe request to the AP waits for its ACK. The identity at
    // 20 MHz takes Linux's 8852B rate checks for the band (5 GHz before any
    // tune), and clearing it brings back no link, sniffer mode and the
    // Bluetooth checks.
    const std::uint8_t ap[6]{0x02, 0x10, 0x20, 0x30, 0x40, 0x50};
    std::vector<std::uint8_t> unicast(24, 0);
    unicast[0] = 0x40;
    std::copy_n(ap, 6, unicast.begin() + 4);
    std::copy_n(self, 6, unicast.begin() + 10);
    const auto unicast_wd = kestrel::build_mgnt_txdesc(unicast.data(), 24, kestrel::TxRate{}, 0, 9,
                                                       kestrel::WD_BODY_LEN, 1, false);
    check(!(word(unicast_wd, 28) & (1u << 11)) && ((word(unicast_wd, 28) >> 25) & 0x3f) == 1,
          "stage-8 probe request: unicast, one attempt");
    const auto net_type = [&] { return (bus->read32(r::R_AX_PORT_CFG_P0) >> r::B_AX_NET_TYPE_P0_SH) & 3; };
    bus->bytes[r::R_AX_TXRATE_CHK] = r::B_AX_BAND_MODE; // the 2.4 GHz checks, to be replaced
    check(hal.set_station_identity(self, ap) && (bus->read8(r::R_AX_TXRATE_CHK) & 0x13) == 0x03 &&
          net_type() == r::MAC_AX_NET_TYPE_ADHOC && !(bus->read32(r::R_AX_RX_FLTR_OPT) & r::B_AX_SNIFFER_MODE),
          "identity at 20 MHz: 5 GHz rate checks, ad hoc port, sniffer off");
    check(hal.clear_station_identity(self) && net_type() == r::MAC_AX_NET_TYPE_NO_LINK &&
          (bus->read32(r::R_AX_RX_FLTR_OPT) & r::B_AX_SNIFFER_MODE) &&
          (bus->read32(r::R_AX_CCA_CONTROL) & r::B_AX_SIFS_CHK_BTCCA) &&
          (bus->read32(r::R_AX_TRXPTCL_RESP_0) & r::B_AX_RSP_CHK_BTCCA),
          "identity cleared: no link, sniffer and Bluetooth checks back");
    check(hal.pcie_deinit() && hal.power_off() && bus->mac_state() == 0, "stop and power-off after the TX setup");
}
// The devourer device orchard-radio creates for a PCIe RTL8852BE
// (DeviceConfig::tuning.kestrel_be_pcie): the station capability only behind
// the flag, no identity before InitWrite, and rtw89's two bring-up passes.
void be_device(Logger_t logger) {
    const devourer::MacAddr own{{0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee}}, bssid{{0x02, 1, 2, 3, 4, 5}};
    for (const bool experimental : {false, true}) {
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot();
        std::size_t written = 0;
        {
            devourer::DeviceConfig config;
            config.tuning.kestrel_be_pcie = experimental;
            RtlKestrelDevice device(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B, config);
            check(device.GetAdapterCaps().station_mode_ok == experimental, "station capability only behind the flag");
            check(!device.SetStationIdentity(own, bssid), "no station identity before InitWrite");
            written = bus->written.size();
        }
        check(bus->written.size() == written && bus->dma_state() == devourer::DmaState::stopped,
              "a device that never started (inspect) leaves without a stop");
    }
    {
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot(); bus->usb = true;
        devourer::DeviceConfig config;
        config.tuning.kestrel_be_pcie = true;
        RtlKestrelDevice device(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B, config);
        check(!device.GetAdapterCaps().station_mode_ok, "the flag does not reach the USB 8852B");
    }
    devourer::DeviceConfig config;
    config.tuning.kestrel_be_pcie = true;
    {
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot(); bus->allow_tx = true;
        program_efuse(*bus);
        RtlKestrelDevice device(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B, config);
        kestrel::EfuseInfo info;
        check(device.PowerOnFwAndTrxBe(info) && info.rfe_type == 5, "BE bring-up passes with the efuse kept");
        check(bus->setups == 2 && bus->drains == 1, "probe pass drained and stopped, then a second download");
        check(bus->mac_state() == 1 && efuse_power_settings(*bus), "start pass powered on with the efuse's settings");
        check(bus->read32(0x1010) == 0 && bus->dma_state() == devourer::DmaState::running,
              "PCIe post-init open, DMA reported running");
    }
    {
        // orchard-radio serve's order on the device, then its destruction.
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot(); bus->allow_tx = true;
        program_efuse(*bus);
        {
            RtlKestrelDevice device(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B, config);
            device.InitWrite({149, 0, CHANNEL_WIDTH_20});
            check(bus->lte[0x38] == 0x77007700, "without the PTA the card's Bluetooth grant is forced low");
            check(device.SetStationIdentity(own, bssid) && (bus->read32(0xce20) & 0x1f) == 0x0f,
                  "after InitWrite the identity filters on A1 and keeps sniffer mode");
            check(device.ClearStationIdentity() && (bus->read32(0xce20) & 0x3f) == 0x3f,
                  "clearing restores the sniffer profile");
        }
        check(bus->dma_state() == devourer::DmaState::stopped && bus->mac_state() == 0,
              "destroying the device stops DMA and powers off");
    }
    // The same with orchard-radio --bt-coex pta. On channel 149 Wi-Fi holds
    // its grant and the Bluetooth keeps the arbiter's. The PTA goes back to the
    // Bluetooth while the MAC still answers, before power-off: through Stop(),
    // which orchard-radio calls first, and through destruction alone.
    for (const bool stop_first : {true, false}) {
        const auto bus = std::make_shared<MemoryTransport>();
        bus->boot(); bus->allow_tx = true;
        program_efuse(*bus);
        auto pta = config;
        pta.tuning.kestrel_hardware_pta = true;
        {
            RtlKestrelDevice device(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B, pta);
            device.InitWrite({149, 0, CHANNEL_WIDTH_20});
            check(bus->lte[0x38] == 0x33003300 && (bus->read8(0x73) & 4) && bus->read16(0xc67c) == 0x100,
                  "PTA on 5 GHz: Wi-Fi grant held, Bluetooth grant left to the arbiter");
            check(device.SetStationIdentity(own, bssid), "the identity is accepted under the PTA");
            if (stop_first) device.Stop();
        }
        check(!(bus->read8(0x73) & 4) && bus->lte[0x38] == 0 && bus->mac_state() == 0,
              stop_first ? "Stop returns the PTA to the Bluetooth before the card powers off"
                         : "destruction returns the PTA to the Bluetooth before the card powers off");
    }
    const auto blank = std::make_shared<MemoryTransport>();
    blank->boot(); blank->allow_tx = true;
    RtlKestrelDevice device(RtlAdapter(blank, logger), logger, kestrel::ChipVariant::C8852B, config);
    kestrel::EfuseInfo info;
    check(!device.PowerOnFwAndTrxBe(info) && blank->setups == 1 && blank->mac_state() == 0,
          "a blank efuse stops after the probe pass, powered off");
}
void station_and_ba(Logger_t logger) {
    const std::uint8_t own[6]{2,1,2,3,4,5}, bssid[6]{2,6,7,8,9,10};
    const auto bus = std::make_shared<MemoryTransport>(); bus->allow_tx = true;
    kestrel::HalKestrel hal(RtlAdapter(bus, logger), logger, kestrel::ChipVariant::C8852B);
    hal.set_hardware_pta(true);
    check(hal.trx_cmac_rx_init(), "station test active PTA");
    bus->write32(0xc390, 0x20200000); bus->write32(0xcc04, 0x82000000);
    bus->write32(0xc400, 0x55550000);
    const auto filter = bus->read32(0xce20);
    bus->tx_fails = true;
    check(!hal.set_station_identity(own, bssid) && bus->read32(0xc400) == 0x55550000 && bus->read32(0xce20) == filter,
          "failed identity H2C does not change port/filter");
    bus->tx_fails = false;
    check(hal.set_station_identity(own, bssid), "identity H2C accepted by fake transport");
    check(bus->read32(0xc390) == 0x20200000 && bus->read32(0xcc04) == 0x82000000, "station identity preserves BTCCA during experimental PTA");
    bus->tx_fails = true;
    const auto port = bus->read32(0xc400);
    check(!hal.clear_station_identity(own) && bus->read32(0xc400) == port, "failed clear H2C leaves port state intact");
    bus->tx_fails = false;
    check(hal.clear_station_identity(own), "identity clear accepted by fake transport");

    for (const auto chip : {kestrel::ChipVariant::C8852B, kestrel::ChipVariant::C8852C}) {
        kestrel::KestrelFw fw(RtlAdapter(bus, logger), logger, chip);
        check(fw.fw_ba_cam(7, 3, 0xabc, 64, 1, true), "BA command sent");
        const auto offset = chip == kestrel::ChipVariant::C8852B ? 32 : 24;
        const std::array<std::uint8_t, 8> b{0x37,7,0xc0,0xab,0,0,0,0};
        const std::array<std::uint8_t, 8> c{0x33,7,0xc0,0xab,0,1,0,0x10};
        const auto& expected = chip == kestrel::ChipVariant::C8852B ? b : c;
        check(bus->sent.size() == offset + 8 && std::equal(expected.begin(), expected.end(), bus->sent.begin()+offset), "B/C BA CAM literal firmware bytes");
        if (chip == kestrel::ChipVariant::C8852B) {
            const auto saved = bus->sent;
            check(!fw.fw_ba_cam(7, 3, 0, 64, 2, true) && bus->sent == saved, "B unsupported static BA entry sends nothing");
        }
    }
}
void release_report_offset() {
    // Short RXD, rpkt_type7 release report, one driver-info unit, 4B payload.
    // A CE-sized driver-info unit must fail against this exact B packet.
    std::array<std::uint8_t, 28> packet{};
    packet[0] = 4; packet[3] = 0x17;
    packet[24] = 0xaa; packet[25] = 0xbb; packet[26] = 0xcc; packet[27] = 0xdd;
    kestrel::KestrelRxFrame frame{};
    check(kestrel::parse_rx_8852b(packet.data(), packet.size(), frame, 8), "B release report parsed");
    check(frame.payload == packet.data() + 24 && frame.payload_len == 4, "B driver-info is eight bytes");
    check(!kestrel::parse_rx_8852b(packet.data(), packet.size(), frame, 16), "CE RX offset rejected for B packet");
}
}
int main() {
    try {
        const auto logger = std::make_shared<Logger>(); logger->set_level(Logger::Level::Error);
        quotas(logger, false, kestrel::ChipVariant::C8852B);
        quotas(logger, true, kestrel::ChipVariant::C8852B);
        quotas(logger, false, kestrel::ChipVariant::C8852C);
        pta(logger, false); pta(logger, true); pcie_lifecycle(logger); stop_contract(logger);
        probe_and_start(logger); be_device(logger); station_and_ba(logger); release_report_offset();
        std::cout << "BE/USB-B/CE HAL quota traces, BE probe, runtime, MAC, BB/RF, TX setup and identity, DMA stop "
                     "contract, BE PCIe device and PTA failure integration passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
