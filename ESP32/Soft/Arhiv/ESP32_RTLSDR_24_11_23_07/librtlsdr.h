#pragma once










//
//int rtlsdr_read_array(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
//int rtlsdr_write_array(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t* array, uint8_t len);
//int rtlsdr_i2c_write_reg(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t reg, uint8_t val);
//uint8_t rtlsdr_i2c_read_reg(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t reg);
//int rtlsdr_i2c_write(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t* buffer, int len);
//int rtlsdr_i2c_read(rtlsdr_dev_t* dev, uint8_t i2c_addr, uint8_t* buffer, int len);
//uint16_t rtlsdr_read_reg(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint8_t len);
//int rtlsdr_write_reg(rtlsdr_dev_t* dev, uint8_t block, uint16_t addr, uint16_t val, uint8_t len);
//uint16_t rtlsdr_demod_read_reg(rtlsdr_dev_t* dev, uint8_t page, uint16_t addr, uint8_t len);
//int rtlsdr_demod_write_reg(rtlsdr_dev_t* dev, uint8_t page, uint16_t addr, uint16_t val, uint8_t len);
//void rtlsdr_set_gpio_bit(rtlsdr_dev_t* dev, uint8_t gpio, int val);
//void rtlsdr_set_gpio_output(rtlsdr_dev_t* dev, uint8_t gpio);
//void rtlsdr_set_i2c_repeater(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_fir(rtlsdr_dev_t* dev);
//void rtlsdr_init_baseband(rtlsdr_dev_t* dev);
//int rtlsdr_deinit_baseband(rtlsdr_dev_t* dev);
//static int rtlsdr_set_if_freq(rtlsdr_dev_t* dev, uint32_t freq);
//int rtlsdr_set_sample_freq_correction(rtlsdr_dev_t* dev, int ppm);
//int rtlsdr_set_xtal_freq(rtlsdr_dev_t* dev, uint32_t rtl_freq, uint32_t tuner_freq);
//int rtlsdr_get_xtal_freq(rtlsdr_dev_t* dev, uint32_t* rtl_freq, uint32_t* tuner_freq);
//int rtlsdr_get_usb_strings(rtlsdr_dev_t* dev, char* manufact, char* product, char* serial);
//int rtlsdr_write_eeprom(rtlsdr_dev_t* dev, uint8_t* data, uint8_t offset, uint16_t len);
//int rtlsdr_read_eeprom(rtlsdr_dev_t* dev, uint8_t* data, uint8_t offset, uint16_t len);
//int rtlsdr_set_center_freq(rtlsdr_dev_t* dev, uint32_t freq);
//uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t* dev);
//int rtlsdr_set_freq_correction(rtlsdr_dev_t* dev, int ppm);
//int rtlsdr_get_freq_correction(rtlsdr_dev_t* dev);
//enum rtlsdr_tuner rtlsdr_get_tuner_type(rtlsdr_dev_t* dev);
//int rtlsdr_get_tuner_gains(rtlsdr_dev_t* dev, int* gains);
//int rtlsdr_set_tuner_bandwidth(rtlsdr_dev_t* dev, uint32_t bw);
//int rtlsdr_set_tuner_gain(rtlsdr_dev_t* dev, int gain);
//int rtlsdr_get_tuner_gain(rtlsdr_dev_t* dev);
//int rtlsdr_set_tuner_if_gain(rtlsdr_dev_t* dev, int stage, int gain);
//int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t* dev, int mode);
//int rtlsdr_set_sample_rate(rtlsdr_dev_t* dev, uint32_t samp_rate);
//uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t* dev);
//int rtlsdr_set_testmode(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_agc_mode(rtlsdr_dev_t* dev, int on);
//int rtlsdr_set_direct_sampling(rtlsdr_dev_t* dev, int on);
//int rtlsdr_get_direct_sampling(rtlsdr_dev_t* dev);
//int rtlsdr_set_offset_tuning(rtlsdr_dev_t* dev, int on);
//int rtlsdr_get_offset_tuning(rtlsdr_dev_t* dev);
//static rtlsdr_dongle_t* find_known_device(uint16_t vid, uint16_t pid);
//void esp_action_get_dev_desc(rtlsdr_dev_t* dev);
//int rtlsdr_open(rtlsdr_dev_t** out_dev, uint8_t index, usb_host_client_handle_t client_hdl);
//int rtlsdr_close(rtlsdr_dev_t* dev);
//int rtlsdr_reset_buffer(rtlsdr_dev_t* dev);
//int rtlsdr_read_sync(rtlsdr_dev_t* dev, void* buf, int len, int* n_read);
//int rtlsdr_cancel_async(rtlsdr_dev_t* dev);
//uint32_t rtlsdr_get_tuner_clock(void* dev);
//int rtlsdr_i2c_write_fn(void* dev, uint8_t addr, uint8_t* buf, int len);
//int rtlsdr_i2c_read_fn(void* dev, uint8_t addr, uint8_t* buf, int len);
//int rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t* dev, int gpio, int on);
//int rtlsdr_set_bias_tee(rtlsdr_dev_t* dev, int on);