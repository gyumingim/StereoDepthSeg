/* idf_stubs.h — MINIMAL host stand-ins for the ESP-IDF APIs used by esp32fc/main, ONLY for a gcc -fsyntax-only check
 * of our own code (typos, types, missing declarations). Signatures follow the ESP-IDF v5.2 documentation as remembered
 * (추정); this is NOT a substitute for `idf.py build`. Every other header in this folder just includes this file. */
#ifndef IDF_STUBS_H
#define IDF_STUBS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110d
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1110
#define ESP_ERROR_CHECK(x) do { esp_err_t e_ = (x); (void)e_; } while (0)
#define IRAM_ATTR
#define RTC_NOINIT_ATTR
/* FreeRTOS */
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
typedef void *QueueHandle_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef struct { int x; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define configMAX_PRIORITIES 25
#define portENTER_CRITICAL(m) (void)(m)
#define portEXIT_CRITICAL(m) (void)(m)
#define portYIELD_FROM_ISR(x) (void)(x)
QueueHandle_t xQueueCreate(UBaseType_t n, UBaseType_t sz);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t t);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t t);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t t);
BaseType_t xTaskNotifyGive(TaskHandle_t t);
void vTaskNotifyGiveFromISR(TaskHandle_t t, BaseType_t *hp);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskCreatePinnedToCore(void (*f)(void *), const char *n, uint32_t st, void *a, UBaseType_t prio, TaskHandle_t *h,
                                   BaseType_t core);
void vTaskDelay(TickType_t t);
void vTaskDelayUntil(TickType_t *prev, TickType_t inc);
TickType_t xTaskGetTickCount(void);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
/* esp_timer / wdt / log */
int64_t esp_timer_get_time(void);
esp_err_t esp_task_wdt_add(TaskHandle_t t);
esp_err_t esp_task_wdt_reset(void);
typedef enum { ESP_LOG_NONE, ESP_LOG_ERROR, ESP_LOG_WARN } esp_log_level_t;
void esp_log_level_set(const char *tag, esp_log_level_t l);
/* gpio */
typedef int gpio_num_t;
typedef enum { GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE, GPIO_PULLUP_ENABLE } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE, GPIO_PULLDOWN_ENABLE } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE, GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE } gpio_int_type_t;
typedef struct { uint64_t pin_bit_mask; gpio_mode_t mode; gpio_pullup_t pull_up_en; gpio_pulldown_t pull_down_en; gpio_int_type_t intr_type; } gpio_config_t;
#define ESP_INTR_FLAG_IRAM (1 << 10)
esp_err_t gpio_config(const gpio_config_t *c);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(gpio_num_t n, void (*h)(void *), void *a);
int gpio_get_level(gpio_num_t n);
esp_err_t gpio_set_level(gpio_num_t n, uint32_t l);
esp_err_t gpio_reset_pin(gpio_num_t n);
esp_err_t gpio_set_direction(gpio_num_t n, gpio_mode_t m);
/* spi */
typedef enum { SPI1_HOST, SPI2_HOST, SPI3_HOST } spi_host_device_t;
#define SPI_DMA_CH_AUTO 3
#define SPI_TRANS_USE_TXDATA (1 << 3)
typedef struct { int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num, max_transfer_sz; } spi_bus_config_t;
typedef struct { int clock_speed_hz; uint8_t mode; int spics_io_num; int queue_size; } spi_device_interface_config_t;
typedef void *spi_device_handle_t;
typedef struct { uint32_t flags; size_t length; const void *tx_buffer; void *rx_buffer; uint8_t tx_data[4]; } spi_transaction_t;
esp_err_t spi_bus_initialize(spi_host_device_t h, const spi_bus_config_t *c, int dma);
esp_err_t spi_bus_add_device(spi_host_device_t h, const spi_device_interface_config_t *c, spi_device_handle_t *d);
esp_err_t spi_device_polling_transmit(spi_device_handle_t d, spi_transaction_t *t);
/* rmt */
typedef void *rmt_channel_handle_t;
typedef void *rmt_encoder_handle_t;
typedef enum { RMT_CLK_SRC_DEFAULT } rmt_clock_source_t;
typedef struct { gpio_num_t gpio_num; rmt_clock_source_t clk_src; uint32_t resolution_hz; size_t mem_block_symbols; size_t trans_queue_depth; } rmt_tx_channel_config_t;
typedef struct { uint16_t duration0 : 15; uint16_t level0 : 1; uint16_t duration1 : 15; uint16_t level1 : 1; } rmt_symbol_word_t;
typedef struct { rmt_symbol_word_t bit0, bit1; struct { uint32_t msb_first : 1; } flags; } rmt_bytes_encoder_config_t;
typedef struct { int loop_count; } rmt_transmit_config_t;
esp_err_t rmt_new_tx_channel(const rmt_tx_channel_config_t *c, rmt_channel_handle_t *h);
esp_err_t rmt_new_bytes_encoder(const rmt_bytes_encoder_config_t *c, rmt_encoder_handle_t *e);
esp_err_t rmt_enable(rmt_channel_handle_t h);
esp_err_t rmt_disable(rmt_channel_handle_t h);
esp_err_t rmt_transmit(rmt_channel_handle_t h, rmt_encoder_handle_t e, const void *p, size_t n, const rmt_transmit_config_t *c);
/* ledc */
typedef enum { LEDC_LOW_SPEED_MODE } ledc_mode_t;
typedef enum { LEDC_TIMER_0 } ledc_timer_t;
typedef enum { LEDC_TIMER_14_BIT = 14 } ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK } ledc_clk_cfg_t;
typedef int ledc_channel_t;
typedef struct { ledc_mode_t speed_mode; ledc_timer_bit_t duty_resolution; ledc_timer_t timer_num; uint32_t freq_hz; ledc_clk_cfg_t clk_cfg; } ledc_timer_config_t;
typedef struct { int gpio_num; ledc_mode_t speed_mode; ledc_channel_t channel; ledc_timer_t timer_sel; uint32_t duty; int hpoint; } ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t *c);
esp_err_t ledc_channel_config(const ledc_channel_config_t *c);
esp_err_t ledc_set_duty(ledc_mode_t m, ledc_channel_t c, uint32_t d);
esp_err_t ledc_update_duty(ledc_mode_t m, ledc_channel_t c);
/* gptimer */
typedef void *gptimer_handle_t;
typedef enum { GPTIMER_CLK_SRC_DEFAULT } gptimer_clock_source_t;
typedef enum { GPTIMER_COUNT_UP } gptimer_count_direction_t;
typedef struct { gptimer_clock_source_t clk_src; gptimer_count_direction_t direction; uint32_t resolution_hz; } gptimer_config_t;
typedef struct { uint64_t count_value, alarm_value; } gptimer_alarm_event_data_t;
typedef bool (*gptimer_alarm_cb_t)(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *);
typedef struct { gptimer_alarm_cb_t on_alarm; } gptimer_event_callbacks_t;
typedef struct { uint64_t alarm_count, reload_count; struct { uint32_t auto_reload_on_alarm : 1; } flags; } gptimer_alarm_config_t;
esp_err_t gptimer_new_timer(const gptimer_config_t *c, gptimer_handle_t *t);
esp_err_t gptimer_register_event_callbacks(gptimer_handle_t t, const gptimer_event_callbacks_t *c, void *u);
esp_err_t gptimer_set_alarm_action(gptimer_handle_t t, const gptimer_alarm_config_t *c);
esp_err_t gptimer_enable(gptimer_handle_t t);
esp_err_t gptimer_start(gptimer_handle_t t);
/* adc */
typedef void *adc_oneshot_unit_handle_t;
typedef void *adc_cali_handle_t;
typedef enum { ADC_UNIT_1 } adc_unit_t;
typedef enum { ADC_CHANNEL_0 } adc_channel_t;
typedef enum { ADC_ATTEN_DB_12 = 3 } adc_atten_t;
typedef enum { ADC_BITWIDTH_DEFAULT } adc_bitwidth_t;
typedef struct { adc_unit_t unit_id; } adc_oneshot_unit_init_cfg_t;
typedef struct { adc_atten_t atten; adc_bitwidth_t bitwidth; } adc_oneshot_chan_cfg_t;
typedef struct { adc_unit_t unit_id; adc_channel_t chan; adc_atten_t atten; adc_bitwidth_t bitwidth; } adc_cali_curve_fitting_config_t;
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *c, adc_oneshot_unit_handle_t *h);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t h, adc_channel_t ch, const adc_oneshot_chan_cfg_t *c);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t h, adc_channel_t ch, int *raw);
esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t *c, adc_cali_handle_t *h);
esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t h, int raw, int *mv);
/* nvs */
typedef uint32_t nvs_handle_t;
typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;
esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);
esp_err_t nvs_open(const char *ns, nvs_open_mode_t m, nvs_handle_t *h);
esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *v, size_t *n);
esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t n);
esp_err_t nvs_commit(nvs_handle_t h);
void nvs_close(nvs_handle_t h);
/* wifi / esp-now */
typedef struct { int dummy; } wifi_init_config_t;
#define WIFI_INIT_CONFIG_DEFAULT() {0}
typedef enum { WIFI_STORAGE_RAM = 1 } wifi_storage_t;
typedef enum { WIFI_MODE_STA = 1 } wifi_mode_t;
typedef enum { WIFI_SECOND_CHAN_NONE } wifi_second_chan_t;
typedef enum { WIFI_PS_NONE } wifi_ps_type_t;
typedef enum { WIFI_IF_STA } wifi_interface_t;
esp_err_t esp_netif_init(void);
esp_err_t esp_event_loop_create_default(void);
esp_err_t esp_wifi_init(const wifi_init_config_t *c);
esp_err_t esp_wifi_set_storage(wifi_storage_t s);
esp_err_t esp_wifi_set_mode(wifi_mode_t m);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_set_channel(uint8_t ch, wifi_second_chan_t s);
esp_err_t esp_wifi_set_ps(wifi_ps_type_t t);
typedef struct { uint8_t *src_addr; uint8_t *des_addr; void *rx_ctrl; } esp_now_recv_info_t;
typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *info, const uint8_t *data, int len);
typedef struct { uint8_t peer_addr[6]; uint8_t lmk[16]; uint8_t channel; wifi_interface_t ifidx; bool encrypt; void *priv; } esp_now_peer_info_t;
esp_err_t esp_now_init(void);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_set_pmk(const uint8_t *pmk);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t *p);
esp_err_t esp_now_send(const uint8_t *addr, const uint8_t *data, size_t len);
/* tinyusb (espressif/esp_tinyusb 1.x) */
typedef struct { const void *device_descriptor; const char **string_descriptor; bool external_phy; } tinyusb_config_t;
typedef enum { TINYUSB_USBDEV_0 } tinyusb_usbdev_t;
typedef enum { TINYUSB_CDC_ACM_0 } tinyusb_cdcacm_itf_t;
typedef struct { int type; union { struct { bool dtr; bool rts; } line_state_changed_data; }; } cdcacm_event_t;
typedef void (*tusb_cdcacm_callback_t)(int itf, cdcacm_event_t *event);
typedef struct { tinyusb_usbdev_t usb_dev; tinyusb_cdcacm_itf_t cdc_port; size_t rx_unread_buf_sz; tusb_cdcacm_callback_t callback_rx, callback_rx_wanted_char, callback_line_state_changed, callback_line_coding_changed; } tinyusb_config_cdcacm_t;
esp_err_t tinyusb_driver_install(const tinyusb_config_t *c);
esp_err_t tusb_cdc_acm_init(const tinyusb_config_cdcacm_t *c);
esp_err_t tinyusb_cdcacm_read(int itf, uint8_t *buf, size_t sz, size_t *rx);
#endif
