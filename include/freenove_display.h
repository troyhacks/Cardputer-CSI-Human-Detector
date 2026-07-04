#ifndef FREENOVE_DISPLAY_H
#define FREENOVE_DISPLAY_H

// =============================================================================
// Freenove FNK0104B (ESP32-S3-WROOM-1) Display + Touch Driver
//
// Hardware (from YAML reference):
//   Display: ILI9341 240x320 4-wire SPI
//     CS=10, DC=46, MOSI=11, SCLK=12, BL=45 (PWM active HIGH)
//   Touch:   FT6336U I2C 0x38
//     SDA=16, SCL=15, INT=17, RST=18
//
// Touch zones (240x320 screen):
//   Zone 0 (top 240px,  y 0-239):    unused (radar display area)
//   Zone 1 (bottom row,  y 240-319, x 0-59):    threshold -
//   Zone 2 (bottom row,  y 240-319, x 60-119):   threshold +
//   Zone 3 (bottom row,  y 240-319, x 120-179):  calibrate
//   Zone 4 (bottom row,  y 240-319, x 180-239):  settings menu
// =============================================================================
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <driver/ledc.h>
#include <FT6336U.h>

// ── Panel: ILI9341 240x320, using built-in init commands ─────────────────────
using FreenovePanel = lgfx::v1::Panel_ILI9341;

// ── Bus: 4-wire SPI for ILI9341 ────────────────────────────────────────────
using FreenoveBus = lgfx::v1::Bus_SPI;

// ── Device class ─────────────────────────────────────────────────────────────
class FreenoveDisplay : public lgfx::v1::LGFX_Device {
public:
    FreenoveDisplay();
    bool begin();
    int pollTouch();
    void setBrightness(uint8_t percent);

private:
    FreenovePanel  _panel;
    FreenoveBus    _bus;
    FT6336U        _touch{16, 15, 18, 17};  // SDA, SCL, RST, INT

    // LEDC backlight
    ledc_channel_config_t _bl_ch{};
    bool _bl_inited = false;

    // Touch state
    int  _last_zone = -1;
    uint32_t _last_touch_ms = 0;
    static constexpr uint32_t DEBOUNCE_MS = 80;
};

// ── Implementation ───────────────────────────────────────────────────────────

FreenoveDisplay::FreenoveDisplay() {
    // SPI bus
    {
        FreenoveBus::config_t cfg{};
        cfg.spi_host   = SPI2_HOST;
        cfg.spi_mode   = 0;
        cfg.freq_write = 40000000;
        cfg.freq_read  = 16000000;
        cfg.spi_3wire  = true;
        cfg.use_lock   = true;
        cfg.dma_channel = 1;
        cfg.pin_sclk  = 12;
        cfg.pin_mosi  = 11;
        cfg.pin_miso  = -1;
        cfg.pin_dc    = 46;
        _bus.config(cfg);
        _panel.setBus(&_bus);
    }

    // Panel
    {
        lgfx::v1::Panel_Device::config_t cfg{};
        cfg.pin_cs      = 10;
        cfg.pin_rst     = -1;   // no hardware reset
        cfg.pin_busy    = -1;
        cfg.memory_width  = 240;
        cfg.memory_height = 320;
        cfg.panel_width   = 240;
        cfg.panel_height  = 320;
        cfg.offset_x      = 0;
        cfg.offset_y      = 0;
        cfg.offset_rotation = 0;   // no panel rotation
        cfg.invert        = true;    // ILI9341 needs color inversion
        cfg.rgb_order     = false;   // BGR
        cfg.bus_shared    = false;
        cfg.dummy_read_pixel = 8;
        cfg.dummy_read_bits  = 1;
        cfg.readable         = false;
        _panel.config(cfg);
    }

    setPanel(&_panel);
}

bool FreenoveDisplay::begin() {
    // LEDC backlight on GPIO 45
    ledc_timer_config_t timer_cfg{
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num      = LEDC_TIMER_0,
        .freq_hz        = 1000,
        .clk_cfg        = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);
    _bl_ch.gpio_num   = (gpio_num_t)45;
    _bl_ch.speed_mode = LEDC_LOW_SPEED_MODE;
    _bl_ch.channel    = LEDC_CHANNEL_0;
    _bl_ch.timer_sel  = LEDC_TIMER_0;
    _bl_ch.duty       = 128;
    _bl_ch.hpoint     = 0;
    ledc_channel_config(&_bl_ch);
    _bl_inited = true;
    setBrightness(80);

    // I2C for FT6336U touch: SDA=16, SCL=15
    Wire.begin(16, 15);
    Wire.setClock(400000);

    // Hardware reset FT6336U before library init
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);
    delay(10);
    digitalWrite(18, HIGH);
    delay(50);
    _touch.begin();

    init();

    return true;
}

int FreenoveDisplay::pollTouch() {
    uint32_t now = millis();

    FT6336U_TouchPointType result = _touch.scan();
    if (result.touch_count == 0) {
        _last_zone = -1;
        return -1;
    }

    TouchPointType& tp = result.tp[0];
    if (tp.status == release) {
        _last_zone = -1;
        return -1;
    }

    if (now - _last_touch_ms < DEBOUNCE_MS) return _last_zone;
    _last_touch_ms = now;

    int x = tp.x;
    int y = tp.y;

    int zone;
    if (y < 240) {
        zone = 0;                                    // top radar area (unused)
    } else {
        // Bottom row: 4 equal horizontal zones (60px each)
        if      (x < 60)  zone = 1;                 // left: threshold -
        else if (x < 120) zone = 2;                 // mid-left: threshold +
        else if (x < 180) zone = 3;                 // mid-right: calibrate
        else              zone = 4;                 // right: settings menu
    }

    _last_zone = zone;
    return zone;
}

void FreenoveDisplay::setBrightness(uint8_t percent) {
    if (!_bl_inited) return;
    uint32_t duty = ((uint32_t)percent * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

#endif // FREENOVE_DISPLAY_H
