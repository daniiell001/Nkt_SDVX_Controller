#include <Arduino.h>
#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ============================================================
// 用户可配置参数
// ============================================================

// ---- 鼠标灵敏度 ----
const float ENCODER_SENSITIVITY_X = 0.020f;
const float ENCODER_SENSITIVITY_Y = 0.020f;
const bool INVERT_X = false;
const bool INVERT_Y = false;

// ---- 编码器死区 ----
const int ENCODER_DEAD_ZONE = 1;
const int DEAD_ZONE_SOFT_END = 3;
const bool SOFT_DEAD_ZONE = false;

// ---- 1€ 滤波器 ----
const float ONE_EURO_FC_MIN = 2.5f;
const float ONE_EURO_BETA = 0.025f;
const float ONE_EURO_FC_D = 1.0f;

// ---- 响应曲线 ----
const float POWER_CURVE = 1.12f;

// ---- 方向消抖 (ms) ----
const int DIRECTION_DEBOUNCE_MS = 3;

// ---- I2C 频率 (Hz) ----
const uint32_t I2C_FREQ = 400000;

const uint32_t LOOP_PERIOD_US = 1000;

// ---- 按钮去抖 (ms) ----
const int BTN_DEBOUNCE_MS = 5;

// ---- NeoPixel ----
const int NEO_PIN = 16;
const int NEO_NUM = 1;
const int NEO_BRI = 40; // 0-255

const uint32_t LED_INIT = 0x0000FF;
const uint32_t LED_IDLE = 0x000000;
const uint32_t LED_PRESS = 0xFF00FF;
const uint32_t LED_STOP = 0xFF0000;

const uint32_t HEARTBEAT_INTERVAL = 1000;

const int BTN_PINS[] = {0, 1, 2, 3, 8, 9, 10};
const int BTN_COUNT = sizeof(BTN_PINS) / sizeof(BTN_PINS[0]);
const char BTN_KEYS[] = {'d', 'f', 'j', 'k', 'c', 'm', 'y'};
//   BT_A='d'  BT_B='f'  BT_C='j'  BT_D='k'
//   FX_L='c'  FX_R='m'  START='y'

struct EncoderState
{
    uint16_t raw_angle;
    uint16_t last_angle;
    bool angle_valid;
    uint32_t seq;
};
static EncoderState g_enc_x, g_enc_y;

struct PipelineState
{
    float accum;
    float xhat;
    float dxhat;
    float prev_raw;
    uint64_t prev_ts_us;
    int last_dir;
    uint64_t debounce_until_us;
};
static PipelineState g_pipe_x, g_pipe_y;

// ---- 按钮 ----
struct ButtonState
{
    int pin;
    char key;
    bool pressed;
    uint32_t last_ms;
    bool debouncing;
};
static ButtonState g_btns[BTN_COUNT];

// ---- LED ----
static Adafruit_NeoPixel g_led(NEO_NUM, NEO_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t g_led_color = LED_IDLE;
static uint32_t g_led_off_ms = 0;
static bool g_led_on = false;

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_REG_RAW = 0x0C;
const uint8_t AS5600_REG_STATUS = 0x0B;

static uint16_t i2c_read_raw_angle(TwoWire &wire, uint8_t addr)
{
    wire.beginTransmission(addr);
    wire.write(AS5600_REG_RAW);
    wire.endTransmission(false);

    wire.requestFrom(addr, (uint8_t)2);
    uint8_t hi = wire.read();
    uint8_t lo = wire.read();

    return ((uint16_t)(hi & 0x0F) << 8) | lo;
}

static bool i2c_init_and_check(TwoWire &wire, const char *name)
{
    wire.begin();
    wire.setClock(I2C_FREQ);

    wire.beginTransmission(AS5600_ADDR);
    wire.write(AS5600_REG_STATUS);
    wire.endTransmission(false);
    wire.requestFrom(AS5600_ADDR, (uint8_t)1);

    if (!wire.available())
    {
        Serial.print("  [");
        Serial.print(name);
        Serial.println("] I2C read failed!");
        return false;
    }

    uint8_t st = wire.read();
    bool md = (st >> 5) & 1;
    bool mh = (st >> 3) & 1;

    if (md && !mh)
    {
        uint16_t angle = i2c_read_raw_angle(wire, AS5600_ADDR);
        Serial.print("  [");
        Serial.print(name);
        Serial.print("] Magnet OK, angle=");
        Serial.println(angle);
        return true;
    }

    Serial.print("  [");
    Serial.print(name);
    Serial.print("] WARNING: Magnet issue (status=0x");
    Serial.print(st, HEX);
    Serial.println(")");
    return false;
}

static void read_both_encoders(uint16_t &out_x, uint16_t &out_y)
{

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_RAW);
    Wire.endTransmission(false);

    Wire1.beginTransmission(AS5600_ADDR);
    Wire1.write(AS5600_REG_RAW);
    Wire1.endTransmission(false);

    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
    Wire1.requestFrom(AS5600_ADDR, (uint8_t)2);

    uint8_t x_hi = Wire.read();
    uint8_t x_lo = Wire.read();
    uint8_t y_hi = Wire1.read();
    uint8_t y_lo = Wire1.read();

    out_x = ((uint16_t)(x_hi & 0x0F) << 8) | x_lo;
    out_y = ((uint16_t)(y_hi & 0x0F) << 8) | y_lo;
}

#define M_PI_f 3.1415926535f

static float apply_dead_zone(float delta)
{
    float abs_d = fabsf(delta);
    if (abs_d < (float)ENCODER_DEAD_ZONE)
        return 0.0f;
    if (SOFT_DEAD_ZONE && abs_d < (float)DEAD_ZONE_SOFT_END)
    {
        float factor = (abs_d - ENCODER_DEAD_ZONE) / (float)(DEAD_ZONE_SOFT_END - ENCODER_DEAD_ZONE);
        return (delta > 0 ? 1.0f : -1.0f) * abs_d * factor;
    }
    return delta;
}

static float apply_one_euro(PipelineState *s, float raw, uint64_t now_us)
{
    if (ONE_EURO_FC_MIN <= 0.0f || ONE_EURO_BETA <= 0.0f)
        return raw;

    if (s->prev_ts_us == 0)
    {
        s->xhat = raw;
        s->dxhat = 0.0f;
        s->prev_raw = raw;
        s->prev_ts_us = now_us;
        return raw;
    }

    float dt = (float)(now_us - s->prev_ts_us) * 1e-6f;
    if (dt < 0.00005f)
        dt = 0.00005f;
    s->prev_ts_us = now_us;

    float alpha_d = dt / (dt + 1.0f / (2.0f * M_PI_f * ONE_EURO_FC_D));
    float speed = (raw - s->prev_raw) / dt;
    s->dxhat = alpha_d * speed + (1.0f - alpha_d) * s->dxhat;

    // 自适应截止频率
    float fc = ONE_EURO_FC_MIN + ONE_EURO_BETA * fabsf(s->dxhat);

    // 平滑信号
    float alpha = dt / (dt + 1.0f / (2.0f * M_PI_f * fc));
    s->xhat = alpha * raw + (1.0f - alpha) * s->xhat;

    s->prev_raw = raw;
    return s->xhat;
}

static float apply_power_curve(float val)
{
    if (POWER_CURVE == 1.0f)
        return val;
    float sign = (val >= 0.0f) ? 1.0f : -1.0f;
    return sign * powf(fabsf(val), POWER_CURVE);
}

static float apply_direction_debounce(PipelineState *s, float dx,
                                      uint64_t now_us)
{
    if (DIRECTION_DEBOUNCE_MS <= 0 || dx == 0.0f)
        return dx;

    int dir = (dx > 0.0f) ? 1 : -1;
    uint64_t debounce_us = (uint64_t)DIRECTION_DEBOUNCE_MS * 1000ULL;

    if (s->last_dir != 0 && dir != s->last_dir)
    {
        if (now_us < s->debounce_until_us)
            return 0.0f;
        s->debounce_until_us = now_us + debounce_us;
        s->last_dir = dir;
    }
    else
    {
        s->last_dir = dir;
        s->debounce_until_us = 0;
    }
    return dx;
}

static float run_pipeline(PipelineState *s, float dx, uint64_t now_us)
{
    dx = apply_dead_zone(dx);
    dx = apply_one_euro(s, dx, now_us);
    dx = apply_power_curve(dx);
    dx = apply_direction_debounce(s, dx, now_us);
    return dx;
}

static int compute_delta(uint16_t raw, uint16_t *last, bool *valid)
{
    if (!*valid)
    {
        *last = raw;
        *valid = true;
        return 0;
    }
    int d = (int)raw - (int)(*last);
    *last = raw;
    if (d > 2048)
        return d - 4096;
    else if (d < -2048)
        return d + 4096;
    return d;
}

static void buttons_init()
{
    for (int i = 0; i < BTN_COUNT; i++)
    {
        g_btns[i].pin = BTN_PINS[i];
        g_btns[i].key = BTN_KEYS[i];
        g_btns[i].pressed = false;
        g_btns[i].last_ms = 0;
        g_btns[i].debouncing = false;
        pinMode(BTN_PINS[i], INPUT_PULLUP);
    }
}

static void buttons_poll()
{
    uint32_t now = millis();

    for (int i = 0; i < BTN_COUNT; i++)
    {
        ButtonState *b = &g_btns[i];
        bool raw = !digitalRead(b->pin);

        if (raw == b->pressed)
        {
            b->debouncing = false;
            continue;
        }

        if (!b->debouncing)
        {
            b->debouncing = true;
            b->last_ms = now;
            continue;
        }

        if (now - b->last_ms >= (uint32_t)BTN_DEBOUNCE_MS)
        {
            b->pressed = raw;
            b->debouncing = false;

            if (raw)
            {
                Keyboard.press(b->key);
            }
            else
            {
                Keyboard.release(b->key);
            }

            if (raw)
            {
                g_led_color = LED_PRESS;
                g_led_on = true;
                g_led_off_ms = millis() + 100;
            }
        }
    }
}

static void led_init()
{
    g_led.begin();
    g_led.setBrightness(NEO_BRI);
    g_led.setPixelColor(0, LED_INIT);
    g_led.show();
}

static void led_update()
{
    if (g_led_on && millis() > g_led_off_ms)
    {
        g_led_color = LED_IDLE;
        g_led_on = false;
    }
    g_led.setPixelColor(0, g_led_color);
    g_led.show();
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n========================================"));
    Serial.println(F("  SDVX Controller — RP2040 + AS5600"));
    Serial.println(F("  [C: Dual HW I2C @ 1000Hz]"));
    Serial.println(F("========================================"));

    // ---- LED ----
    led_init();

    // ---- I2C + 编码器 ----
    Serial.println(F("\n[I2C] Initializing dual I2C buses..."));
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire1.setSDA(6);
    Wire1.setSCL(7);

    bool ok_x = i2c_init_and_check(Wire, "X (I2C0 GP4/GP5)");
    bool ok_y = i2c_init_and_check(Wire1, "Y (I2C1 GP6/GP7)");

    if (!ok_x || !ok_y)
    {
        Serial.println(F("  WARNING: One or both encoders may not work."));
    }

    // ---- 按钮 ----
    Serial.println(F("\n[Buttons]"));
    buttons_init();
    Serial.print(F("  "));
    Serial.print(BTN_COUNT);
    Serial.println(F(" buttons"));

    // ---- USB HID ----
    Serial.println(F("\n[HID] Starting USB HID..."));
    Mouse.begin();
    Keyboard.begin();
    Serial.println(F("  Mouse + Keyboard ready"));

    g_led_color = LED_IDLE;
    led_update();

    Serial.println(F("\n========================================"));
    Serial.println(F("  Ready! @ 1000Hz I2C read loop"));
    Serial.println(F("========================================\n"));
}

void loop()
{
    static uint32_t next_loop_us = 0;
    static uint32_t loop_count = 0;
    static uint64_t last_seq_x = 0;
    static uint64_t last_seq_y = 0;

    uint32_t now_us = micros();
    if (now_us < next_loop_us)
    {
        delayMicroseconds(50);
        return;
    }
    next_loop_us = now_us + LOOP_PERIOD_US;
    loop_count++;
    uint64_t now_us64 = (uint64_t)now_us;

    led_update();

    uint16_t raw_x, raw_y;
    read_both_encoders(raw_x, raw_y);

    int dx = compute_delta(raw_x, &g_enc_x.last_angle, &g_enc_x.angle_valid);
    int dy = compute_delta(raw_y, &g_enc_y.last_angle, &g_enc_y.angle_valid);

    if (INVERT_X)
        dx = -dx;
    if (INVERT_Y)
        dy = -dy;

    float fx = run_pipeline(&g_pipe_x, (float)dx, now_us64);
    float fy = run_pipeline(&g_pipe_y, (float)dy, now_us64);

    g_pipe_x.accum += fx * ENCODER_SENSITIVITY_X;
    g_pipe_y.accum += fy * ENCODER_SENSITIVITY_Y;

    int mx = (int)g_pipe_x.accum;
    int my = (int)g_pipe_y.accum;

    if (mx != 0 || my != 0)
    {
        Mouse.move(mx, my);
        g_pipe_x.accum -= (float)mx;
        g_pipe_y.accum -= (float)my;
    }

    buttons_poll();

    if (loop_count % HEARTBEAT_INTERVAL == 0)
    {
        Serial.print(F("  ["));
        Serial.print(loop_count);
        Serial.print(F("] raw=("));
        Serial.print(raw_x);
        Serial.print(F(","));
        Serial.print(raw_y);
        Serial.print(F(")  d=("));
        Serial.print(dx);
        Serial.print(F(","));
        Serial.print(dy);
        Serial.print(F(")  f=("));
        Serial.print(fx, 1);
        Serial.print(F(","));
        Serial.print(fy, 1);
        Serial.print(F(")  acc=("));
        Serial.print(g_pipe_x.accum, 1);
        Serial.print(F(","));
        Serial.print(g_pipe_y.accum, 1);
        Serial.println(F(")"));
    }
}
