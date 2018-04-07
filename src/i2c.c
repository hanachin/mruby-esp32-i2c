#include <mruby.h>
#include <mruby/array.h>
#include <mruby/class.h>
#include <mruby/data.h>
#include <mruby/hash.h>
#include <mruby/string.h>
#include <mruby/value.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

/* https://github.com/espressif/esp-idf/blob/595ddfd8250426f2e76f7c47ffe9697eb13c455a/examples/peripherals/i2c/main/i2c_example_main.c#L65 */
#define MRUBY_ESP32_I2C_DEFAULT_SLAVE_ADDR 0x28
#define MRUBY_ESP32_I2C_DEFAULT_FREQ 100000
/*
 * https://github.com/espressif/arduino-esp32/blob/25dff4f044151f7f766c64b9d2ad90398472e6b3/variants/esp32/pins_arduino.h#L17,L18
 */
#define MRUBY_ESP32_I2C_DEFAULT_SDA GPIO_NUM_21
#define MRUBY_ESP32_I2C_DEFAULT_SCL GPIO_NUM_22

typedef struct mrb_esp32_i2c {
  i2c_port_t i2c_num;
  i2c_mode_t mode;
  size_t slv_rx_buf_len;
  size_t slv_tx_buf_len;
  int intr_alloc_flags;
} mrb_esp32_i2c;

typedef struct mrb_esp32_i2c_isr_arg {
  mrb_state *mrb;
  mrb_value block;
  intr_handle_t intr_handle;
} mrb_esp32_i2c_isr_arg;

static const struct mrb_data_type mrb_esp32_i2c_type = {
  "mrb_esp32_i2c", mrb_free
};

static const struct mrb_data_type mrb_esp32_i2c_intr_handle_type = {
  "i2c_intr_handle_t", mrb_free
};

static const struct mrb_data_type mrb_esp32_i2c_config_type = {
  "i2c_config_t", mrb_free
};

static const struct mrb_data_type mrb_esp32_i2c_cmd_handle_type = {
  "i2c_cmd_handle_t", mrb_free
};

/*
 * ESP32::I2C
 */

/*
 * initialize ESP32::I2C
 *
 *     i2c = ESP32::I2C.new(ESP32::I2C::NUM_0, ESP32::I2C::MODE_MASTER, 0, 0, 0)
 */
static mrb_value mrb_esp32_i2c_init(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c;
  mrb_int i2c_num, mode, slv_rx_buf_len, slv_tx_buf_len, intr_alloc_flags;

  mrb_get_args(mrb, "iiiii", &i2c_num, &mode, &slv_rx_buf_len, &slv_tx_buf_len, &intr_alloc_flags);

  /* avoid memory leaks */
  i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  if (i2c) {
    mrb_free(mrb, i2c);
  }
  mrb_data_init(self, NULL, &mrb_esp32_i2c_type);

  /* create mrb_esp_i2c */
  i2c = (mrb_esp32_i2c *)mrb_malloc(mrb, sizeof(mrb_esp32_i2c));

  i2c->i2c_num = (i2c_port_t)i2c_num;
  i2c->mode = (i2c_mode_t)mode;
  i2c->slv_rx_buf_len = (size_t)slv_rx_buf_len;
  i2c->slv_tx_buf_len = (size_t)slv_tx_buf_len;
  i2c->intr_alloc_flags = (int)intr_alloc_flags;

  mrb_data_init(self, i2c, &mrb_esp32_i2c_type);

  return self;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv218i2c_driver_install10i2c_port_t10i2c_mode_t6size_t6size_ti i2c_driver_install}
 *
 *     i2c.driver_install
 */
static mrb_value mrb_esp32_i2c_driver_install(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_driver_install(i2c->i2c_num, i2c->mode, i2c->slv_rx_buf_len, i2c->slv_tx_buf_len, i2c->intr_alloc_flags));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv217i2c_driver_delete10i2c_port_t i2c_driver_delete}
 *
 *     i2c.driver_delete
 */
static mrb_value mrb_esp32_i2c_driver_delete(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_driver_delete(i2c->i2c_num));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv216i2c_param_config10i2c_port_tPK12i2c_config_t i2c_param_config}
 *
 *     conf = ESP32::I2C::Config.new
 *     i2c.param_config(conf)
 */
static mrb_value mrb_esp32_i2c_param_config(mrb_state *mrb, mrb_value self) {
  mrb_value i2c_config;
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  i2c_config_t *i2c_conf;
  mrb_get_args(mrb, "o", i2c_config);
  i2c_conf = (i2c_config_t *)DATA_PTR(i2c_config);
  return mrb_fixnum_value((mrb_int)i2c_param_config(i2c->i2c_num, i2c_conf));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv217i2c_reset_tx_fifo10i2c_port_t i2c_reset_tx_fifo}
 *
 *     i2c.reset_tx_fifo
 */
static mrb_value mrb_esp32_i2c_reset_tx_fifo(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_reset_tx_fifo(i2c->i2c_num));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv217i2c_reset_rx_fifo10i2c_port_t i2c_reset_rx_fifo}
 *
 *     i2c.reset_rx_fifo
 */
static mrb_value mrb_esp32_i2c_reset_rx_fifo(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_reset_rx_fifo(i2c->i2c_num));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv211i2c_set_pin10i2c_port_tii13gpio_pullup_t13gpio_pullup_t10i2c_mode_t i2c_set_pin}
 *
 *   # Add hanachin/mruby-esp32-gpio to conf.gem to use ESP32::GPIO
 *   sda_io_num = ESP32::GPIO::NUM_0
 *   scl_io_num = ESP32::GPIO::NUM_1
 *   sda_pullup_en = scl_pullup_en = ESP32::GPIO::PULLUP_ENABLE
 *   i2c.set_pin(sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en, ESP32::I2C::MODE_MASTER)
 */
static mrb_value mrb_esp32_i2c_set_pin(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en, mode;
  mrb_get_args(mrb, "iiiiii", &sda_io_num, &scl_io_num, &sda_pullup_en, &scl_pullup_en, &mode);
  return mrb_fixnum_value((mrb_int)i2c_set_pin(i2c->i2c_num, (gpio_num_t)sda_io_num, (gpio_num_t)scl_io_num, (gpio_pullup_t)sda_pullup_en, (gpio_pullup_t)scl_pullup_en, (i2c_mode_t)mode));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv222i2c_slave_write_buffer10i2c_port_tP7uint8_ti10TickType_t i2c_slave_write_buffer}
 *
 *     ticks_to_wait = 1234
 *     i2c.slave_write_buffer("hi", 1234)
 *
*/
static mrb_value mrb_esp32_i2c_slave_write_buffer(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  char *s;
  mrb_int size, ticks_to_wait;
  mrb_get_args(mrb, "si", &s, &size, &ticks_to_wait);
  return mrb_fixnum_value((mrb_int)i2c_slave_write_buffer(i2c->i2c_num, (uint8_t *)s, (int)size, (TickType_t)ticks_to_wait));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv221i2c_slave_read_buffer10i2c_port_tP7uint8_t6size_t10TickType_t i2c_slave_read_buffer}
 *
 *     ticks_to_wait = 1234
 *     i2c.slave_read_buffer(1234, ticks_to_wait)
 */
static mrb_value mrb_esp32_i2c_slave_read_buffer(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_value buf;
  mrb_int max_size, ticks_to_wait;
  int size;
  mrb_get_args(mrb, "ii", &max_size, &ticks_to_wait);
  buf = mrb_str_new(mrb, NULL, max_size);
  size = i2c_slave_read_buffer(i2c->i2c_num, (uint8_t *)RSTRING_PTR(buf), (int)max_size, (TickType_t)ticks_to_wait);
  return mrb_str_resize(mrb, buf, size);
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv214i2c_set_period10i2c_port_tii i2c_set_period}
 *
 *      high_period = low_period = 1234
 *      i2c.set_period(high_period, low_period)
 */
static mrb_value mrb_esp32_i2c_set_period(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int high_period, low_period;
  mrb_get_args(mrb, "ii", &high_period, &low_period);
  return mrb_fixnum_value(i2c_set_period(i2c->i2c_num, (int)high_period, (int)low_period));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv214i2c_get_period10i2c_port_tPiPi i2c_get_period}
 *
 *     ret, high_period, low_period = i2c.get_period
 */
static mrb_value mrb_esp32_i2c_get_period(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int high_period, low_period;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_period(i2c->i2c_num, &high_period, &low_period)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(high_period));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(low_period));
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv220i2c_set_start_timing10i2c_port_tii i2c_set_start_timing}
 *
 *     setup_time = hold_time = 1234
 *     i2c.set_start_timing(setup_time, hold_time)
 */
static mrb_value mrb_esp32_i2c_set_start_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int setup_time, hold_time;
  mrb_get_args(mrb, "ii", &setup_time, &hold_time);
  return mrb_fixnum_value(i2c_set_start_timing(i2c->i2c_num, (int)setup_time, (int)hold_time));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv220i2c_get_start_timing10i2c_port_tPiPi i2c_get_start_timing}
 *
 *     ret, setup_time, hold_time = i2c.get_start_timing
 */
static mrb_value mrb_esp32_i2c_get_start_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int setup_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_start_timing(i2c->i2c_num, &setup_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(setup_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_set_stop_timing10i2c_port_tii i2c_set_stop_timing}
 *
 *     setup_time = hold_time = 1234
 *     i2c.set_stop_timing(setup_time, hold_time)
 */
static mrb_value mrb_esp32_i2c_set_stop_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int setup_time, hold_time;
  mrb_get_args(mrb, "ii", &setup_time, &hold_time);
  return mrb_fixnum_value(i2c_set_stop_timing(i2c->i2c_num, (int)setup_time, (int)hold_time));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_get_stop_timing10i2c_port_tPiPi i2c_get_stop_timing}
 *
 *     ret, setup_time, hold_time = i2c.get_stop_timing
 */
static mrb_value mrb_esp32_i2c_get_stop_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int setup_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_stop_timing(i2c->i2c_num, &setup_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(setup_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/*
 *  {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_set_data_timing10i2c_port_tii set_data_timing}
 *
 *     sample_time = hold_time = 1234
 *     i2c.set_data_timing(sample_time, hold_time)
 */
static mrb_value mrb_esp32_i2c_set_data_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int sample_time, hold_time;
  mrb_get_args(mrb, "ii", &sample_time, &hold_time);
  return mrb_fixnum_value(i2c_set_data_timing(i2c->i2c_num, (int)sample_time, (int)hold_time));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_get_data_timing10i2c_port_tPiPi i2c_get_data_timing}
 *
 *     ret, sample_time, hold_time = i2c.get_data_timing
 */
static mrb_value mrb_esp32_i2c_get_data_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int sample_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_data_timing(i2c->i2c_num, &sample_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(sample_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv215i2c_set_timeout10i2c_port_ti i2c_set_timeout}
 *
 *     i2c.set_timeout(1234)
 */
static mrb_value mrb_esp32_i2c_set_timeout(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int timeout;
  mrb_get_args(mrb, "i", &timeout);
  return mrb_fixnum_value(i2c_set_timeout(i2c->i2c_num, (int)timeout));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv215i2c_get_timeout10i2c_port_tPi i2c_get_timeout}
 *
 *      ret, timeout = i2c.get_timeout
 */
static mrb_value mrb_esp32_i2c_get_timeout(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int timeout;
  mrb_value ary = mrb_ary_new_capa(mrb, 2);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_timeout(i2c->i2c_num, &timeout)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(timeout));
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv217i2c_set_data_mode10i2c_port_t16i2c_trans_mode_t16i2c_trans_mode_t i2c_set_data_mode}
 *
 *     tx_trans_mode = rx_trans_mode = ESP32::I2C::DATA_MODE_MSB_FIRST
 *     i2c.set_data_mode(tx_trans_mode, rx_trans_mode)
 */
static mrb_value mrb_esp32_i2c_set_data_mode(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int tx_trans_mode, rx_trans_mode;
  mrb_get_args(mrb, "ii", &tx_trans_mode, &rx_trans_mode);
  return mrb_fixnum_value(i2c_set_data_mode(i2c->i2c_num, (i2c_trans_mode_t)tx_trans_mode, (i2c_trans_mode_t)rx_trans_mode));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv217i2c_get_data_mode10i2c_port_tP16i2c_trans_mode_tP16i2c_trans_mode_t i2c_get_data_mode}
 *
 *     ret, tx_trans_mode, rx_trans_mode = i2c.get_data_mode
 */
static mrb_value mrb_esp32_i2c_get_data_mode(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  i2c_trans_mode_t tx_trans_mode, rx_trans_mode;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_data_mode(i2c->i2c_num, &tx_trans_mode, &rx_trans_mode)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value((mrb_int)tx_trans_mode));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value((mrb_int)rx_trans_mode));
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv216i2c_master_start16i2c_cmd_handle_t i2c_master_cmd_begin}
 *
 *     ticks_to_wait = 1234
 *     cmd = ESP32::I2C::CmdHandle.new
 *     cmd.master_start
 *     # ...
 *     cmd.master_stop
 *     i2c.master_cmd_begin(cmd, ticks_to_wait)
 */
static mrb_value mrb_esp32_i2c_master_cmd_begin(mrb_state *mrb, mrb_value self) {
  mrb_value cmd;
  mrb_int ticks_to_wait;
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_get_args(mrb, "oi", &cmd, &ticks_to_wait);
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(cmd);
  return mrb_fixnum_value(i2c_master_cmd_begin(i2c->i2c_num, *cmd_handle, (TickType_t)ticks_to_wait));
}

static void mrb_esp32_i2c_isr_handler(void* arg) {
  mrb_esp32_i2c_isr_arg *a = (mrb_esp32_i2c_isr_arg *)arg;
  mrb_funcall(a->mrb, a->block, "call", 0);
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv216i2c_isr_register10i2c_port_tPFvPvEPviP13intr_handle_t i2c_isr_register}
 *
 *     ret, intr_handle = i2c.isr_register(0) do
 *       # interrupt
 *     end
 */
static mrb_value mrb_esp32_i2c_isr_register(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_value block, ary, intr_handle;
  mrb_esp32_i2c_isr_arg *arg;
  int intr_alloc_flags;
  esp_err_t ret;
  struct RClass *mrb_esp32, *mrb_esp32_i2c, *mrb_esp32_i2c_intr_handle;

  mrb_get_args(mrb, "i&", &intr_alloc_flags, &block);

  arg = (mrb_esp32_i2c_isr_arg *)mrb_malloc(mrb, sizeof(mrb_esp32_i2c_isr_arg));
  arg->mrb = mrb;
  arg->block = block;

  ret = i2c_isr_register(i2c->i2c_num, mrb_esp32_i2c_isr_handler, arg, intr_alloc_flags, &arg->intr_handle);

  if (ret == ESP_OK) {
    mrb_gc_register(mrb, block);

    mrb_esp32 = mrb_define_module(mrb, "ESP32");
    mrb_esp32_i2c = mrb_define_class_under(mrb, mrb_esp32, "I2C", mrb->object_class);
    mrb_esp32_i2c_intr_handle = mrb_define_class_under(mrb, mrb_esp32_i2c, "IntrHandle", mrb->object_class);

    intr_handle = mrb_obj_new(mrb, mrb_esp32_i2c_intr_handle, 0, NULL);
    mrb_data_init(intr_handle, arg, &mrb_esp32_i2c_intr_handle_type);

    ary = mrb_ary_new_capa(mrb, 2);
    mrb_ary_set(mrb, ary, 0, mrb_fixnum_value((mrb_int)ret));
    mrb_ary_set(mrb, ary, 1, intr_handle);
  } else {
    ary = mrb_ary_new_capa(mrb, 2);
    mrb_ary_set(mrb, ary, 0, mrb_fixnum_value((mrb_int)ret));
    mrb_ary_set(mrb, ary, 1, mrb_nil_value());
  }

  return ary;
}

/*
 * ESP32::I2C::IntrHandle
 */

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv212i2c_isr_free13intr_handle_t i2c_isr_free}
 *
 *     ret, intr_handle = i2c.isr_register(0) do
 *       # interrupt
 *     end
 *     intr_handle.free if ret == ESP32::OK
 */
static mrb_value mrb_esp32_i2c_intr_handle_free(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c_isr_arg *arg;
  esp_err_t ret;
  arg = (mrb_esp32_i2c_isr_arg *)DATA_PTR(self);
  ret = i2c_isr_free(arg->intr_handle);
  mrb_gc_unregister(mrb, arg->block);
  mrb_data_init(self, NULL, &mrb_esp32_i2c_intr_handle_type);
  return mrb_fixnum_value((mrb_int)ret);
}

/*
 * ESP32::I2C::Config
 */
static i2c_mode_t option_to_i2c_mode_t(mrb_state *mrb, mrb_value opt, const char *key, i2c_mode_t def) {
  mrb_value mode = mrb_hash_fetch(mrb, opt, mrb_symbol_value(mrb_intern_cstr(mrb, key)), mrb_fixnum_value(def));
  mrb_assert(mrb_fixnum_p(mode));
  return (i2c_mode_t)mrb_fixnum(mode);
}

static gpio_num_t option_to_gpio_num_t(mrb_state *mrb, mrb_value opt, const char *key, gpio_num_t def) {
  mrb_value gpio_num = mrb_hash_fetch(mrb, opt, mrb_symbol_value(mrb_intern_cstr(mrb, key)), mrb_fixnum_value(def));
  mrb_assert(mrb_fixnum_p(gpio_num));
  return (gpio_num_t)mrb_fixnum(gpio_num);
}

static gpio_pullup_t option_to_gpio_pullup_t(mrb_state *mrb, mrb_value opt, const char *key, gpio_pullup_t def) {
  mrb_value gpio_pullup = mrb_hash_fetch(mrb, opt, mrb_symbol_value(mrb_intern_cstr(mrb, key)), mrb_fixnum_value(def));
  mrb_assert(mrb_fixnum_p(gpio_pullup));
  return (gpio_pullup_t)mrb_fixnum(gpio_pullup);
}

static i2c_addr_mode_t option_to_addr_mode(mrb_state *mrb, mrb_value opt, const char *key, i2c_addr_mode_t def) {
  mrb_value addr_mode = mrb_hash_fetch(mrb, opt, mrb_symbol_value(mrb_intern_cstr(mrb, key)), mrb_fixnum_value(def));
  mrb_assert(mrb_fixnum_p(addr_mode));
  return (i2c_addr_mode_t)mrb_fixnum(addr_mode);
}

static mrb_int option_to_mrb_int(mrb_state *mrb, mrb_value opt, const char *key, mrb_int def) {
  mrb_value n = mrb_hash_fetch(mrb, opt, mrb_symbol_value(mrb_intern_cstr(mrb, key)), mrb_fixnum_value(def));
  mrb_assert(mrb_fixnum_p(n));
  return mrb_fixnum(n);
}

/*
 * initialize {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv212i2c_config_t ESP32::I2C::Config}
 *
 *     # Add hanachin/mruby-esp32-gpio to conf.gem to use ESP32::GPIO
 *     config = ESP32::I2C::Config.new(
 *       mode:          ESP32::I2C::MODE_MASTER,
 *       sda_io_num:    ESP32::I2C::DEFAULT_SDA,
 *       sda_pullup_en: ESP32::GPIO::PULLUP_ENABLE,
 *       scl_io_num:    ESP32::I2C::DEFAULT_SCL,
 *       scl_pullup_en: ESP32::GPIO::PULLUP_ENABLE,
 *       clk_speed:     ESP32::I2C::DEFAULT_FREQ,
 *       addr_10bit_en: ESP32::I2C::ADDR_BIT_7,
 *       slave_addr:    ESP32::I2C::DEFAULT_SLAVE_ADDR
 *     )
 */
static mrb_value mrb_esp32_i2c_config_init(mrb_state *mrb, mrb_value self) {
  i2c_config_t *i2c_config;
  mrb_value opt;

  if (mrb_get_args(mrb, "|H", &opt) == 0) {
    opt = mrb_hash_new(mrb);
  }

  /* avoid memory leaks */
  i2c_config = (i2c_config_t *)DATA_PTR(self);
  if (i2c_config) {
    mrb_free(mrb, i2c_config);
  }
  mrb_data_init(self, NULL, &mrb_esp32_i2c_config_type);

  /* create i2c_config_t */
  i2c_config = (i2c_config_t *)mrb_malloc(mrb, sizeof(i2c_config_t));

  i2c_config->mode = option_to_i2c_mode_t(mrb, opt, "mode", I2C_MODE_MASTER);
  i2c_config->sda_io_num = option_to_gpio_num_t(mrb, opt, "sda_io_num", MRUBY_ESP32_I2C_DEFAULT_SDA);
  i2c_config->scl_io_num = option_to_gpio_num_t(mrb, opt, "scl_io_num", MRUBY_ESP32_I2C_DEFAULT_SCL);
  i2c_config->sda_pullup_en = option_to_gpio_pullup_t(mrb, opt, "sda_pullup_en", GPIO_PULLUP_ENABLE);
  i2c_config->scl_pullup_en = option_to_gpio_pullup_t(mrb, opt, "scl_pullup_en", GPIO_PULLUP_ENABLE);

  if (i2c_config->mode == I2C_MODE_MASTER) {
    i2c_config->master.clk_speed = option_to_mrb_int(mrb, opt, "clk_speed", MRUBY_ESP32_I2C_DEFAULT_FREQ);
  } else {
    i2c_config->slave.addr_10bit_en = option_to_addr_mode(mrb, opt, "addr_10bit_en", I2C_ADDR_BIT_7);
    i2c_config->slave.slave_addr = option_to_mrb_int(mrb, opt, "slave_addr", MRUBY_ESP32_I2C_DEFAULT_SLAVE_ADDR);
  }

  mrb_data_init(self, i2c_config, &mrb_esp32_i2c_config_type);

  return self;
}

/*
 * ESP32::I2C::CmdHandle
 */

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_cmd_link_createv i2c_cmd_link_create}
 *
 *     cmd = ESP32::I2C::CmdHandle.new
 */
static mrb_value mrb_esp32_i2c_cmd_handle_init(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *i2c_cmd_handle;

  /* avoid memory leaks */
  i2c_cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  if (i2c_cmd_handle) {
    mrb_free(mrb, i2c_cmd_handle);
  }
  mrb_data_init(self, NULL, &mrb_esp32_i2c_cmd_handle_type);

  /* create i2c_cmd_handle_t */
  i2c_cmd_handle = (i2c_cmd_handle_t *)mrb_malloc(mrb, sizeof(i2c_cmd_handle_t));
  *i2c_cmd_handle = i2c_cmd_link_create();
  mrb_data_init(self, i2c_cmd_handle, &mrb_esp32_i2c_cmd_handle_type);

  return self;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv219i2c_cmd_link_delete16i2c_cmd_handle_t i2c_cmd_link_delete}
 *
 *     cmd = ESP32::I2C::CmdHandle.new
 *     # ...
 *     cmd.delete
 */
static mrb_value mrb_esp32_i2c_cmd_handle_delete(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  i2c_cmd_link_delete(*cmd_handle);
  return mrb_nil_value();
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv216i2c_master_start16i2c_cmd_handle_t i2c_master_start}
 *
 *     cmd.master_start
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_start(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_master_start(*cmd_handle));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv221i2c_master_write_byte16i2c_cmd_handle_t7uint8_tb i2c_master_write_byte}
 *
 *     ack_en = true
 *     cmd.master_write_byte("a", ack_en)
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_write_byte(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  mrb_value s;
  mrb_bool ack_en;
  mrb_get_args(mrb, "Sb", &s, &ack_en);
  return mrb_fixnum_value((mrb_int)i2c_master_write_byte(*cmd_handle, (uint8_t)RSTRING_PTR(s)[0], (bool)ack_en));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv216i2c_master_write16i2c_cmd_handle_tP7uint8_t6size_tb i2c_master_write}
 *
 *     ack_en = true
 *     cmd.master_write("hi", ack_en)
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_write(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  char *data;
  mrb_int data_len;
  mrb_bool ack_en;
  mrb_get_args(mrb, "sb", &data, &data_len, &ack_en);
  return mrb_fixnum_value((mrb_int)i2c_master_write(*cmd_handle, (uint8_t *)data, (size_t)data_len, (bool)ack_en));
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv220i2c_master_read_byte16i2c_cmd_handle_tP7uint8_ti i2c_master_read_byte}
 *
 *     ret, data = cmd.master_read_byte(ESP32::I2C::MASTER_ACK)
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_read_byte(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  mrb_value ary, s;
  mrb_int ack;
  esp_err_t ret;
  mrb_get_args(mrb, "i", &ack);
  s = mrb_str_new(mrb, NULL, 1);
  ret = i2c_master_read_byte(*cmd_handle, RSTRING_PTR(s), (i2c_ack_type_t)ack);
  ary = mrb_ary_new_capa(mrb, 2);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value((mrb_int)ret));
  mrb_ary_set(mrb, ary, 1, s);
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv215i2c_master_read16i2c_cmd_handle_tP7uint8_t6size_ti i2c_master_read}
 *
 *      ret, data = cmd.master_read(1234, ESP32::I2C::MASTER_ACK)
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_read(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  mrb_value ary, s;
  esp_err_t ret;
  mrb_int data_len, ack;
  mrb_get_args(mrb, "ii", &data_len, &ack);
  s = mrb_str_new(mrb, NULL, data_len);
  ret = i2c_master_read(*cmd_handle, (uint8_t *)RSTRING_PTR(s), (size_t)data_len, (i2c_ack_type_t)ack);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value((mrb_int)ret));
  mrb_ary_set(mrb, ary, 1, s);
  return ary;
}

/*
 * {http://esp-idf.readthedocs.io/en/v3.0-rc1/api-reference/peripherals/i2c.html#_CPPv215i2c_master_stop16i2c_cmd_handle_t i2c_master_stop}
 *
 *     cmd.master_stop
 */
static mrb_value mrb_esp32_i2c_cmd_handle_master_stop(mrb_state *mrb, mrb_value self) {
  i2c_cmd_handle_t *cmd_handle = (i2c_cmd_handle_t *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_master_stop(*cmd_handle));
}

/*
 * mruby-esp32-i2c
 */
void mrb_esp32_i2c_gem_init(mrb_state* mrb) {
  struct RClass *mrb_esp32, *mrb_esp32_i2c, *mrb_esp32_intr, *mrb_i2c_intr_handle, *mrb_esp32_i2c_config, *mrb_esp32_i2c_cmd_handle;

  /* ESP32 */
  mrb_esp32 = mrb_define_module(mrb, "ESP32");

  /* ESP32::I2C */
  mrb_esp32_i2c = mrb_define_class_under(mrb, mrb_esp32, "I2C", mrb->object_class);
  MRB_SET_INSTANCE_TT(mrb_esp32_i2c, MRB_TT_DATA);
  mrb_define_method(mrb, mrb_esp32_i2c, "initialize", mrb_esp32_i2c_init, MRB_ARGS_REQ(5));
  mrb_define_method(mrb, mrb_esp32_i2c, "driver_install", mrb_esp32_i2c_driver_install, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "driver_delete", mrb_esp32_i2c_driver_delete, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "param_config", mrb_esp32_i2c_param_config, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrb_esp32_i2c, "reset_tx_fifo", mrb_esp32_i2c_reset_tx_fifo, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "reset_rx_fifo", mrb_esp32_i2c_reset_rx_fifo, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_pin", mrb_esp32_i2c_set_pin, MRB_ARGS_REQ(5));
  mrb_define_method(mrb, mrb_esp32_i2c, "slave_write_buffer", mrb_esp32_i2c_slave_write_buffer, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "slave_read_buffer", mrb_esp32_i2c_slave_read_buffer, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "set_period", mrb_esp32_i2c_set_period, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_period", mrb_esp32_i2c_get_period, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_start_timing", mrb_esp32_i2c_set_start_timing, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_start_timing", mrb_esp32_i2c_get_start_timing, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_stop_timing", mrb_esp32_i2c_set_stop_timing, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_stop_timing", mrb_esp32_i2c_get_stop_timing, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_data_timing", mrb_esp32_i2c_set_data_timing, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_data_timing", mrb_esp32_i2c_get_data_timing, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_timeout", mrb_esp32_i2c_set_timeout, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_timeout", mrb_esp32_i2c_get_timeout, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "set_data_mode", mrb_esp32_i2c_set_data_mode, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "get_data_mode", mrb_esp32_i2c_get_data_mode, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c, "master_cmd_begin", mrb_esp32_i2c_master_cmd_begin, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c, "isr_register", mrb_esp32_i2c_isr_register, MRB_ARGS_REQ(1) | MRB_ARGS_BLOCK());

  /* ESP32::I2C::IntrHandle */
  mrb_i2c_intr_handle = mrb_define_class_under(mrb, mrb_esp32_i2c, "IntrHandle", mrb->object_class);
  mrb_define_method(mrb, mrb_i2c_intr_handle, "free", mrb_esp32_i2c_intr_handle_free, MRB_ARGS_NONE());

  /* ESP32::Intr */
  mrb_esp32_intr = mrb_define_class_under(mrb, mrb_esp32, "Intr", mrb->object_class);

  /* esp_intr_alloc.h */
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL1", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL1));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL2", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL2));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL3", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL3));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL4", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL4));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL5", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL5));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVEL6", mrb_fixnum_value(ESP_INTR_FLAG_LEVEL6));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_NMI", mrb_fixnum_value(ESP_INTR_FLAG_NMI));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_SHARED", mrb_fixnum_value(ESP_INTR_FLAG_SHARED));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_EDGE", mrb_fixnum_value(ESP_INTR_FLAG_EDGE));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_IRAM", mrb_fixnum_value(ESP_INTR_FLAG_IRAM));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_INTRDISABLED", mrb_fixnum_value(ESP_INTR_FLAG_INTRDISABLED));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LOWMED", mrb_fixnum_value(ESP_INTR_FLAG_LOWMED));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_HIGH", mrb_fixnum_value(ESP_INTR_FLAG_HIGH));
  mrb_define_const(mrb, mrb_esp32_intr, "FLAG_LEVELMASK", mrb_fixnum_value(ESP_INTR_FLAG_LEVELMASK));

  /* ESP32::I2C::Config */
  mrb_esp32_i2c_config = mrb_define_class_under(mrb, mrb_esp32_i2c, "Config", mrb-> object_class);
  MRB_SET_INSTANCE_TT(mrb_esp32_i2c_config, MRB_TT_DATA);
  mrb_define_method(mrb, mrb_esp32_i2c_config, "initialize", mrb_esp32_i2c_config_init, MRB_ARGS_OPT(1));

  /* i2c_mode_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "MODE_SLAVE", mrb_fixnum_value(I2C_MODE_SLAVE));
  mrb_define_const(mrb, mrb_esp32_i2c, "MODE_MASTER", mrb_fixnum_value(I2C_MODE_MASTER));
  mrb_define_const(mrb, mrb_esp32_i2c, "MODE_MAX", mrb_fixnum_value(I2C_MODE_MAX));

  /* i2c_rw_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_WRITE", mrb_fixnum_value(I2C_MASTER_WRITE));
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_READ", mrb_fixnum_value(I2C_MASTER_READ));

  /* i2c_trans_mode_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "DATA_MODE_MSB_FIRST", mrb_fixnum_value(I2C_DATA_MODE_MSB_FIRST));
  mrb_define_const(mrb, mrb_esp32_i2c, "DATA_MODE_LSB_FIRST", mrb_fixnum_value(I2C_DATA_MODE_LSB_FIRST));
  mrb_define_const(mrb, mrb_esp32_i2c, "DATA_MODE_MAX", mrb_fixnum_value(I2C_DATA_MODE_MAX));

  /* i2c_opmode_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "CMD_RESTART", mrb_fixnum_value(I2C_CMD_RESTART));
  mrb_define_const(mrb, mrb_esp32_i2c, "CMD_WRITE", mrb_fixnum_value(I2C_CMD_WRITE));
  mrb_define_const(mrb, mrb_esp32_i2c, "CMD_READ", mrb_fixnum_value(I2C_CMD_READ));
  mrb_define_const(mrb, mrb_esp32_i2c, "CMD_STOP", mrb_fixnum_value(I2C_CMD_STOP));
  mrb_define_const(mrb, mrb_esp32_i2c, "CMD_END", mrb_fixnum_value(I2C_CMD_END));

  /* i2c_port_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "NUM_0", mrb_fixnum_value(I2C_NUM_0));
  mrb_define_const(mrb, mrb_esp32_i2c, "NUM_1", mrb_fixnum_value(I2C_NUM_1));
  mrb_define_const(mrb, mrb_esp32_i2c, "NUM_MAX", mrb_fixnum_value(I2C_NUM_MAX));

  /* i2c_addr_mode_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "ADDR_BIT_7", mrb_fixnum_value(I2C_ADDR_BIT_7));
  mrb_define_const(mrb, mrb_esp32_i2c, "ADDR_BIT_10", mrb_fixnum_value(I2C_ADDR_BIT_10));
  mrb_define_const(mrb, mrb_esp32_i2c, "ADDR_BIT_MAX", mrb_fixnum_value(I2C_ADDR_BIT_MAX));

  /* i2c_ack_type_t */
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_ACK", mrb_fixnum_value(I2C_MASTER_ACK));
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_NACK", mrb_fixnum_value(I2C_MASTER_NACK));
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_LAST_NACK", mrb_fixnum_value(I2C_MASTER_LAST_NACK));
  mrb_define_const(mrb, mrb_esp32_i2c, "MASTER_ACK_MAX", mrb_fixnum_value(I2C_MASTER_ACK_MAX));

  /* default values */
  mrb_define_const(mrb, mrb_esp32_i2c, "DEFAULT_SLAVE_ADDR", mrb_fixnum_value(MRUBY_ESP32_I2C_DEFAULT_SLAVE_ADDR));
  mrb_define_const(mrb, mrb_esp32_i2c, "DEFAULT_FREQ", mrb_fixnum_value(MRUBY_ESP32_I2C_DEFAULT_FREQ));
  mrb_define_const(mrb, mrb_esp32_i2c, "DEFAULT_SDA", mrb_fixnum_value(MRUBY_ESP32_I2C_DEFAULT_SDA));
  mrb_define_const(mrb, mrb_esp32_i2c, "DEFAULT_SCL", mrb_fixnum_value(MRUBY_ESP32_I2C_DEFAULT_SCL));
  mrb_define_const(mrb, mrb_esp32_i2c, "DEFAULT_TICKS_TO_WAIT", mrb_fixnum_value(1000 / portTICK_RATE_MS));

  /* ESP32::I2C::CmdHandle */
  mrb_esp32_i2c_cmd_handle = mrb_define_class_under(mrb, mrb_esp32_i2c, "CmdHandle", mrb-> object_class);
  MRB_SET_INSTANCE_TT(mrb_esp32_i2c_cmd_handle, MRB_TT_DATA);
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "initialize", mrb_esp32_i2c_cmd_handle_init, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "delete", mrb_esp32_i2c_cmd_handle_delete, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_start", mrb_esp32_i2c_cmd_handle_master_start, MRB_ARGS_NONE());
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_write_byte", mrb_esp32_i2c_cmd_handle_master_write_byte, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_write", mrb_esp32_i2c_cmd_handle_master_write, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_read_byte", mrb_esp32_i2c_cmd_handle_master_read_byte, MRB_ARGS_REQ(1));
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_read", mrb_esp32_i2c_cmd_handle_master_read, MRB_ARGS_REQ(2));
  mrb_define_method(mrb, mrb_esp32_i2c_cmd_handle, "master_stop", mrb_esp32_i2c_cmd_handle_master_stop, MRB_ARGS_NONE());
}

void mrb_esp32_i2c_gem_final(mrb_state* mrb) {
}
