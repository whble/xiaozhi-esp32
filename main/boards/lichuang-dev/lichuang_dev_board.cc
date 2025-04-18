#include "wifi_board.h"
#include "audio_codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "i2c_device.h"
#include "iot/thing_manager.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <wifi_station.h>

#define TAG "LichuangDevBoard"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);
class Qmi8658a : public I2cDevice
{
    public:
        // QMI8658寄存器地址
        enum qmi8658_reg
        {
            QMI8658_WHO_AM_I,
            QMI8658_REVISION_ID,
            QMI8658_CTRL1,
            QMI8658_CTRL2,
            QMI8658_CTRL3,
            QMI8658_CTRL4,
            QMI8658_CTRL5,
            QMI8658_CTRL6,
            QMI8658_CTRL7,
            QMI8658_CTRL8,
            QMI8658_CTRL9,
            QMI8658_CATL1_L,
            QMI8658_CATL1_H,
            QMI8658_CATL2_L,
            QMI8658_CATL2_H,
            QMI8658_CATL3_L,
            QMI8658_CATL3_H,
            QMI8658_CATL4_L,
            QMI8658_CATL4_H,
            QMI8658_FIFO_WTM_TH,
            QMI8658_FIFO_CTRL,
            QMI8658_FIFO_SMPL_CNT,
            QMI8658_FIFO_STATUS,
            QMI8658_FIFO_DATA,
            QMI8658_STATUSINT = 45,
            QMI8658_STATUS0,
            QMI8658_STATUS1,
            QMI8658_TIMESTAMP_LOW,
            QMI8658_TIMESTAMP_MID,
            QMI8658_TIMESTAMP_HIGH,
            QMI8658_TEMP_L,
            QMI8658_TEMP_H,
            QMI8658_AX_L,
            QMI8658_AX_H,
            QMI8658_AY_L,
            QMI8658_AY_H,
            QMI8658_AZ_L,
            QMI8658_AZ_H,
            QMI8658_GX_L,
            QMI8658_GX_H,
            QMI8658_GY_L,
            QMI8658_GY_H,
            QMI8658_GZ_L,
            QMI8658_GZ_H,
            QMI8658_COD_STATUS = 70,
            QMI8658_dQW_L = 73,
            QMI8658_dQW_H,
            QMI8658_dQX_L,
            QMI8658_dQX_H,
            QMI8658_dQY_L,
            QMI8658_dQY_H,
            QMI8658_dQZ_L,
            QMI8658_dQZ_H,
            QMI8658_dVX_L,
            QMI8658_dVX_H,
            QMI8658_dVY_L,
            QMI8658_dVY_H,
            QMI8658_dVZ_L,
            QMI8658_dVZ_H,
            QMI8658_TAP_STATUS = 89,
            QMI8658_STEP_CNT_LOW,
            QMI8658_STEP_CNT_MIDL,
            QMI8658_STEP_CNT_HIGH,
            QMI8658_RESET = 96
        };
        // 倾角结构体
        struct{
            int16_t acc_x;
            int16_t acc_y;
            int16_t acc_z;
            int16_t gyr_x;
            int16_t gyr_y;
            int16_t gyr_z;
            float AngleX;
            float AngleY;
            float AngleZ;
        }t_sQMI8658;

        Qmi8658a(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        if(0x05==ReadReg(QMI8658_WHO_AM_I))
        {
            ESP_LOGI(TAG, "QMI8658 OK!");  // 打印信息
        }
        else
        {
            ESP_LOGI(TAG, "QMI8658 Not Detected!"); 
        }
    }
};

class Ft6336 : public I2cDevice {
public:
    struct TouchPoint_t {
        int num = 0;
        int x = -1;
        int y = -1;
    };
    
    Ft6336(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t chip_id = ReadReg(0xA3);
        ESP_LOGI(TAG, "Get chip ID: 0x%02X", chip_id);
        read_buffer_ = new uint8_t[6];
    }

    ~Ft6336() {
        delete[] read_buffer_;
    }

    void UpdateTouchPoint() {
        ReadRegs(0x02, read_buffer_, 6);
        tp_.num = read_buffer_[0] & 0x0F;
        tp_.x = ((read_buffer_[1] & 0x0F) << 8) | read_buffer_[2];
        tp_.y = ((read_buffer_[3] & 0x0F) << 8) | read_buffer_[4];
    }

    inline const TouchPoint_t& GetTouchPoint() {
        return tp_;
    }

private:
    uint8_t* read_buffer_ = nullptr;
    TouchPoint_t tp_;
};

class Pca9557 : public I2cDevice {
public:
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        WriteReg(0x01, 0x03);
        WriteReg(0x03, 0xf8);
    }

    void SetOutputState(uint8_t bit, uint8_t level) {
        uint8_t data = ReadReg(0x01);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(0x01, data);
    }
};


class LichuangDevBoard : public WifiBoard {
private:
    uint8_t vol_buf;
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    LcdDisplay* display_;
    Pca9557* pca9557_;
    Ft6336* ft6336_;
    esp_timer_handle_t touchpad_timer_;
    Qmi8658a* Qmi8658a_;

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = (i2c_port_t)1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x19);
    }

    void PollTouchpad() {
        static bool was_touched = false;
        static int64_t touch_start_time = 0;
        const int64_t TOUCH_THRESHOLD_MS = 500;  // 触摸时长阈值，超过500ms视为长按
        
        ft6336_->UpdateTouchPoint();
        auto& touch_point = ft6336_->GetTouchPoint();
        
        // 检测触摸开始
        if (touch_point.num > 0 && !was_touched) {
            was_touched = true;
            touch_start_time = esp_timer_get_time() / 1000; // 转换为毫秒
        } 
        // 检测触摸释放
        else if (touch_point.num == 0 && was_touched) {
            was_touched = false;
            int64_t touch_duration = (esp_timer_get_time() / 1000) - touch_start_time;
            
            // 只有短触才触发
            if (touch_duration < TOUCH_THRESHOLD_MS) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateStarting && 
                    !WifiStation::GetInstance().IsConnected()) {
                    ResetWifiConfiguration();
                }
                app.ToggleChatState();
            }
            else
            {
                uint8_t temp = GetAudioCodec()->output_volume();
                if(temp == 0)
                {
                    GetAudioCodec()->SetOutputVolume(vol_buf);
                }
                else
                {
                    vol_buf = temp;
                    GetAudioCodec()->SetOutputVolume(0);
                }
            }
        }
    }

    void InitializeFt6336TouchPad() {
        ESP_LOGI(TAG, "Init FT6336");
        ft6336_ = new Ft6336(i2c_bus_, 0x38);
        
        // 创建定时器，20ms 间隔
        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                LichuangDevBoard* board = (LichuangDevBoard*)arg;
                board->PollTouchpad();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "touchpad_timer",
            .skip_unhandled_events = true,
        };
        
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &touchpad_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(touchpad_timer_, 20 * 1000));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = GPIO_NUM_40;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = GPIO_NUM_41;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = GPIO_NUM_NC;
        io_config.dc_gpio_num = GPIO_NUM_39;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        pca9557_->SetOutputState(0, 0);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = font_emoji_64_init(),
#endif
                                    });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Camera"));
    }

public:
    LichuangDevBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        InitializeIot();
        InitializeFt6336TouchPad();
        GetBacklight()->RestoreBrightness();
        vol_buf = GetAudioCodec()->output_volume();
        Qmi8658a_ = new Qmi8658a(i2c_bus_, 0x6A);
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            GPIO_NUM_NC, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }
};

DECLARE_BOARD(LichuangDevBoard);
