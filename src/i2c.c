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

/* https://github.com/espressif/esp-idf/blob/595ddfd8250426f2e76f7c47ffe9697eb13c455a/examples/peripherals/i2c/main/i2c_example_main.c#L65 */
#define MRUBY_ESP32_I2C_DEFAULT_SLAVE_ADDR 0x28
#define MRUBY_ESP32_I2C_DEFAULT_FREQ 100000
/*
 * https://github.com/espressif/arduino-esp32/blob/25dff4f044151f7f766c64b9d2ad90398472e6b3/variants/esp32/pins_arduino.h#L17,L18
 */
#define MRUBY_ESP32_I2C_DEFAULT_SDA GPIO_NUM_21
#define MRUBY_ESP32_I2C_DEFAULT_SCL GPIO_NUM_22

/*
 * ESP32::I2C
 */
typedef struct mrb_esp32_i2c {
  i2c_port_t i2c_num;
  i2c_mode_t mode;
  size_t slv_rx_buf_len;
  size_t slv_tx_buf_len;
  int intr_alloc_flags;
} mrb_esp32_i2c;

static const struct mrb_data_type mrb_esp32_i2c_type = {
  "mrb_esp32_i2c", mrb_free
};

/* #initialize */
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

/* #driver_install */
static mrb_value mrb_esp32_i2c_driver_install(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_driver_install(i2c->i2c_num, i2c->mode, i2c->slv_rx_buf_len, i2c->slv_tx_buf_len, i2c->intr_alloc_flags));
}

/* #driver_delete */
static mrb_value mrb_esp32_i2c_driver_delete(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_driver_delete(i2c->i2c_num));
}

/* #param_config */

static mrb_value mrb_esp32_i2c_param_config(mrb_state *mrb, mrb_value self) {
  mrb_value i2c_config;
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  i2c_config_t *i2c_conf;
  mrb_get_args(mrb, "o", i2c_config);
  i2c_conf = (i2c_config_t *)DATA_PTR(i2c_config);
  return mrb_fixnum_value((mrb_int)i2c_param_config(i2c->i2c_num, i2c_conf));
}

/* #reset_tx_fifo */
static mrb_value mrb_esp32_i2c_reset_tx_fifo(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_reset_tx_fifo(i2c->i2c_num));
}

/* #reset_rx_fifo */
static mrb_value mrb_esp32_i2c_reset_rx_fifo(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  return mrb_fixnum_value((mrb_int)i2c_reset_rx_fifo(i2c->i2c_num));
}

/* #set_pin */
static mrb_value mrb_esp32_i2c_set_pin(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int sda_io_num, scl_io_num, sda_pullup_en, scl_pullup_en, mode;
  mrb_get_args(mrb, "iiiiii", &sda_io_num, &scl_io_num, &sda_pullup_en, &scl_pullup_en, &mode);
  return mrb_fixnum_value((mrb_int)i2c_set_pin(i2c->i2c_num, (gpio_num_t)sda_io_num, (gpio_num_t)scl_io_num, (gpio_pullup_t)sda_pullup_en, (gpio_pullup_t)scl_pullup_en, (i2c_mode_t)mode));
}

/* #slave_write_buffer */
static mrb_value mrb_esp32_i2c_slave_write_buffer(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  char *s;
  mrb_int size, ticks_to_wait;
  mrb_get_args(mrb, "si", &s, &size, &ticks_to_wait);
  return mrb_fixnum_value((mrb_int)i2c_slave_write_buffer(i2c->i2c_num, (uint8_t *)s, (int)size, (TickType_t)ticks_to_wait));
}

/* #slave_read_buffer */
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

/* #set_period */
static mrb_value mrb_esp32_i2c_set_period(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int high_period, low_period;
  mrb_get_args(mrb, "ii", &high_period, &low_period);
  return mrb_fixnum_value(i2c_set_period(i2c->i2c_num, (int)high_period, (int)low_period));
}

/* #get_period */
static mrb_value mrb_esp32_i2c_get_period(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int high_period, low_period;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_period(i2c->i2c_num, &high_period, &low_period)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(high_period));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(low_period));
  return ary;
}

/* #set_start_timing */
static mrb_value mrb_esp32_i2c_set_start_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int setup_time, hold_time;
  mrb_get_args(mrb, "ii", &setup_time, &hold_time);
  return mrb_fixnum_value(i2c_set_start_timing(i2c->i2c_num, (int)setup_time, (int)hold_time));
}

/* #get_start_timing */
static mrb_value mrb_esp32_i2c_get_start_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int setup_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_start_timing(i2c->i2c_num, &setup_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(setup_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/* #set_start_timing */
static mrb_value mrb_esp32_i2c_set_stop_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int setup_time, hold_time;
  mrb_get_args(mrb, "ii", &setup_time, &hold_time);
  return mrb_fixnum_value(i2c_set_stop_timing(i2c->i2c_num, (int)setup_time, (int)hold_time));
}

/* #get_start_timing */
static mrb_value mrb_esp32_i2c_get_stop_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int setup_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_stop_timing(i2c->i2c_num, &setup_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(setup_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/* #set_data_timing */
static mrb_value mrb_esp32_i2c_set_data_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int sample_time, hold_time;
  mrb_get_args(mrb, "ii", &sample_time, &hold_time);
  return mrb_fixnum_value(i2c_set_data_timing(i2c->i2c_num, (int)sample_time, (int)hold_time));
}

/* #get_data_timing */
static mrb_value mrb_esp32_i2c_get_data_timing(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int sample_time, hold_time;
  mrb_value ary = mrb_ary_new_capa(mrb, 3);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_data_timing(i2c->i2c_num, &sample_time, &hold_time)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(sample_time));
  mrb_ary_set(mrb, ary, 2, mrb_fixnum_value(hold_time));
  return ary;
}

/* #set_timeout */
static mrb_value mrb_esp32_i2c_set_timeout(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int timeout;
  mrb_get_args(mrb, "i", &timeout);
  return mrb_fixnum_value(i2c_set_timeout(i2c->i2c_num, (int)timeout));
}

/* #get_timeout */
static mrb_value mrb_esp32_i2c_get_timeout(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  int timeout;
  mrb_value ary = mrb_ary_new_capa(mrb, 2);
  mrb_ary_set(mrb, ary, 0, mrb_fixnum_value(i2c_get_timeout(i2c->i2c_num, &timeout)));
  mrb_ary_set(mrb, ary, 1, mrb_fixnum_value(timeout));
  return ary;
}

/* #set_data_mode */
static mrb_value mrb_esp32_i2c_set_data_mode(mrb_state *mrb, mrb_value self) {
  mrb_esp32_i2c *i2c = (mrb_esp32_i2c *)DATA_PTR(self);
  mrb_int tx_trans_mode, rx_trans_mode;
  mrb_get_args(mrb, "ii", &tx_trans_mode, &rx_trans_mode);
  return mrb_fixnum_value(i2c_set_data_mode(i2c->i2c_num, (i2c_trans_mode_t)tx_trans_mode, (i2c_trans_mode_t)rx_trans_mode));
}

/* #get_data_mode */
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
 * ESP32::I2C::Config
 */
static const struct mrb_data_type mrb_esp32_i2c_config_type = {
  "i2c_config_t", mrb_free
};

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
 * mruby-esp32-i2c
 */
void mrb_esp32_i2c_gem_init(mrb_state* mrb) {
  struct RClass *mrb_esp32, *mrb_esp32_i2c, *mrb_esp32_i2c_config, *mrb_esp32_gpio;

  /* ESP32 */
  mrb_esp32 = mrb_define_module(mrb, "ESP32");

  /* TODO move to mruby-esp32 */
  /* esp_err_t */
  mrb_define_const(mrb, mrb_esp32, "OK", mrb_fixnum_value(ESP_OK));
  mrb_define_const(mrb, mrb_esp32, "FAIL", mrb_fixnum_value(ESP_FAIL));
  mrb_define_const(mrb, mrb_esp32, "ERR_NO_MEM", mrb_fixnum_value(ESP_ERR_NO_MEM));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_ARG", mrb_fixnum_value(ESP_ERR_INVALID_ARG));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_STATE", mrb_fixnum_value(ESP_ERR_INVALID_STATE));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_SIZE", mrb_fixnum_value(ESP_ERR_INVALID_SIZE));
  mrb_define_const(mrb, mrb_esp32, "ERR_NOT_FOUND", mrb_fixnum_value(ESP_ERR_NOT_FOUND));
  mrb_define_const(mrb, mrb_esp32, "ERR_NOT_SUPPORTED", mrb_fixnum_value(ESP_ERR_NOT_SUPPORTED));
  mrb_define_const(mrb, mrb_esp32, "ERR_TIMEOUT", mrb_fixnum_value(ESP_ERR_TIMEOUT));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_RESPONSE", mrb_fixnum_value(ESP_ERR_INVALID_RESPONSE));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_CRC", mrb_fixnum_value(ESP_ERR_INVALID_CRC));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_VERSION", mrb_fixnum_value(ESP_ERR_INVALID_VERSION));
  mrb_define_const(mrb, mrb_esp32, "ERR_INVALID_MAC", mrb_fixnum_value(ESP_ERR_INVALID_MAC));
  mrb_define_const(mrb, mrb_esp32, "ERR_WIFI_BASE", mrb_fixnum_value(ESP_ERR_WIFI_BASE));

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

  /* TODO move to mruby-esp32-gpio */
  /* ESP32::GPIO */
  mrb_esp32_gpio = mrb_define_class_under(mrb, mrb_esp32, "GPIO", mrb->object_class);

  /* gpio_num_t */
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_0", mrb_fixnum_value(GPIO_NUM_0));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_1", mrb_fixnum_value(GPIO_NUM_1));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_2", mrb_fixnum_value(GPIO_NUM_2));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_3", mrb_fixnum_value(GPIO_NUM_3));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_4", mrb_fixnum_value(GPIO_NUM_4));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_5", mrb_fixnum_value(GPIO_NUM_5));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_6", mrb_fixnum_value(GPIO_NUM_6));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_7", mrb_fixnum_value(GPIO_NUM_7));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_8", mrb_fixnum_value(GPIO_NUM_8));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_9", mrb_fixnum_value(GPIO_NUM_9));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_10", mrb_fixnum_value(GPIO_NUM_10));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_11", mrb_fixnum_value(GPIO_NUM_11));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_12", mrb_fixnum_value(GPIO_NUM_12));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_13", mrb_fixnum_value(GPIO_NUM_13));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_14", mrb_fixnum_value(GPIO_NUM_14));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_15", mrb_fixnum_value(GPIO_NUM_15));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_16", mrb_fixnum_value(GPIO_NUM_16));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_17", mrb_fixnum_value(GPIO_NUM_17));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_18", mrb_fixnum_value(GPIO_NUM_18));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_19", mrb_fixnum_value(GPIO_NUM_19));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_21", mrb_fixnum_value(GPIO_NUM_21));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_22", mrb_fixnum_value(GPIO_NUM_22));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_23", mrb_fixnum_value(GPIO_NUM_23));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_25", mrb_fixnum_value(GPIO_NUM_25));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_26", mrb_fixnum_value(GPIO_NUM_26));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_27", mrb_fixnum_value(GPIO_NUM_27));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_32", mrb_fixnum_value(GPIO_NUM_32));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_33", mrb_fixnum_value(GPIO_NUM_33));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_34", mrb_fixnum_value(GPIO_NUM_34));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_35", mrb_fixnum_value(GPIO_NUM_35));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_36", mrb_fixnum_value(GPIO_NUM_36));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_37", mrb_fixnum_value(GPIO_NUM_37));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_38", mrb_fixnum_value(GPIO_NUM_38));
  mrb_define_const(mrb, mrb_esp32_gpio, "NUM_39", mrb_fixnum_value(GPIO_NUM_39));

  /* gpio_pullup_t */
  mrb_define_const(mrb, mrb_esp32_gpio, "PULLUP_DISABLE", mrb_fixnum_value(GPIO_PULLUP_DISABLE));
  mrb_define_const(mrb, mrb_esp32_gpio, "PULLUP_ENABLE", mrb_fixnum_value(GPIO_PULLUP_ENABLE));
}

void mrb_esp32_i2c_gem_final(mrb_state* mrb) {
}
