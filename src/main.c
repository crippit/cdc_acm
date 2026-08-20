#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h> 
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <zephyr/random/random.h>

/* USB Headers */
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/class/usb_audio.h>

/* Bluetooth & BAP Headers */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/lc3.h>
#include <lc3.h> 

/* Hardware Bus Headers */
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(zip_audio_router, LOG_LEVEL_INF);

#define HW_SAMPLE_FREQ       48000  
#define SAMPLES_PER_FRAME    480    
#define STEREO_BLOCK_SIZE    1920   
#define NUM_BLOCKS           10      

#define USB_FRAME_SIZE       192    
#define USB_TX_BLOCK_SIZE    256     
#define USB_TX_NUM_BLOCKS    128

#define RING_BUF_SIZE        15360   
RING_BUF_DECLARE(audio_ringbuf, RING_BUF_SIZE);

/* --- Auracast Globals --- */
/* EXPERT FIX: 2 Streams for the "Stereo Spoof" */
static struct bt_bap_stream streams[2];
static struct bt_bap_broadcast_source *broadcast_source;
static struct bt_le_ext_adv *adv;
static uint16_t seq_num = 0;

const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static uint32_t active_bcast_id = 0;

static lc3_encoder_t lc3_encoder;
#define LC3_ENCODER_MAX_MEM 8192
static uint8_t lc3_encoder_mem[LC3_ENCODER_MAX_MEM];

/* Buffer size doubled to 32 to handle 2 streams simultaneously */
NET_BUF_POOL_FIXED_DEFINE(bis_tx_pool, 32, BT_ISO_SDU_BUF_SIZE(100), 8, NULL);
K_MEM_SLAB_DEFINE(i2s_rx_slab, STEREO_BLOCK_SIZE, NUM_BLOCKS, 4);
NET_BUF_POOL_DEFINE(usb_tx_pool, USB_TX_NUM_BLOCKS, USB_TX_BLOCK_SIZE, 0, NULL);
K_SEM_DEFINE(hardware_ready_sem, 0, 1);

/* EXPERT FIX: Codec returned to FRONT_LEFT | FRONT_RIGHT to satisfy Pixel PACS matching */
static struct bt_audio_codec_cfg lc3_codec_cfg = BT_AUDIO_CODEC_LC3_CONFIG(
    BT_AUDIO_CODEC_CFG_FREQ_48KHZ,
    BT_AUDIO_CODEC_CFG_DURATION_10,
    (BT_AUDIO_LOCATION_FRONT_LEFT | BT_AUDIO_LOCATION_FRONT_RIGHT), 
    100, 1, BT_AUDIO_CONTEXT_TYPE_MEDIA
);

static struct bt_bap_qos_cfg lc3_qos_cfg = {
    .phy = BT_BAP_QOS_CFG_2M,
    .framing = BT_BAP_QOS_CFG_FRAMING_UNFRAMED,
    .rtn = 2,
    .sdu = 100,
    .latency = 65,       
    .interval = 10000,   
    .pd = 40000,         
};

const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
const struct device *const i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
const struct device *mic_dev; 

#define SW0_NODE DT_ALIAS(sw0)   
#define LED0_NODE DT_ALIAS(led0) 
#define LED1_NODE DT_ALIAS(led1) 

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec led_line = GPIO_DT_SPEC_GET(LED0_NODE, gpios); 
static const struct gpio_dt_spec led_mic = GPIO_DT_SPEC_GET(LED1_NODE, gpios);  

static struct gpio_callback button_cb_data;
static bool is_line_active = true; 
static volatile bool request_input_switch = false;
static uint32_t last_press_time = 0;

void cdc_id_pulse_handler(struct k_timer *timer_id) {
    if (active_bcast_id != 0 && device_is_ready(uart_dev)) {
        char buf[32];
        snprintf(buf, sizeof(buf), "ID:%06X\n", active_bcast_id);
        for (int i = 0; buf[i] != '\0'; i++) {
            uart_poll_out(uart_dev, buf[i]);
        }
    }
}
K_TIMER_DEFINE(usb_pulse_timer, cdc_id_pulse_handler, NULL);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    uint32_t now = k_uptime_get_32();
    if (now - last_press_time > 250) { 
        request_input_switch = true;
        last_press_time = now;
    }
}

/* --- SGTL5000 I2C Configuration --- */
#define SGTL5000_ADDR 0x0A

static int sgtl5000_write(uint16_t reg, uint16_t val) {
    uint8_t buf[4] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return i2c_write(i2c_bus, buf, 4, SGTL5000_ADDR);
}

static void wake_sgtl5000(void) {
    if (!device_is_ready(i2c_bus)) return;
    k_sleep(K_MSEC(50));
    sgtl5000_write(0x0002, 0x0000);
    sgtl5000_write(0x0030, 0x4060); 
    sgtl5000_write(0x0026, 0x006C); 
    sgtl5000_write(0x0028, 0x01F2); 
    sgtl5000_write(0x002C, 0x0F22); 
    sgtl5000_write(0x003C, 0x4446); 
    sgtl5000_write(0x0030, 0x40FF); 
    sgtl5000_write(0x0002, 0x0073); 
    k_sleep(K_MSEC(400));
    sgtl5000_write(0x0004, 0x0008);
    
    /* SGTL5000 in SLAVE mode (0x0130) because nRF5340 is the Master */
    sgtl5000_write(0x0006, 0x0130);
    
    sgtl5000_write(0x000A, 0x0000);
    sgtl5000_write(0x000E, 0x00B0); 
    sgtl5000_write(0x0024, 0x0004);
    sgtl5000_write(0x0020, 0x0000); 
    sgtl5000_write(0x002A, 0x0000); 
    sgtl5000_write(0x0022, 0x2020);
}

/* --- USB Audio Callbacks --- */
static int16_t last_sample[2] = {0, 0}; 

static void data_received_cb(const struct device *dev, struct net_buf *buffer, size_t size) {}
static void data_written_cb(const struct device *dev, struct net_buf *buffer, size_t size) { net_buf_unref(buffer); }
static void feature_update_cb(const struct device *dev, const struct usb_audio_fu_evt *evt) {}

static void data_request_cb(const struct device *dev)
{
    size_t frame_size = 192; 
    struct net_buf *buf = net_buf_alloc(&usb_tx_pool, K_NO_WAIT);
    if (!buf) return;

    uint8_t temp_buf[256]; 
    uint32_t data_in_buffer = ring_buf_size_get(&audio_ringbuf);

    if (data_in_buffer >= frame_size) {
        ring_buf_get(&audio_ringbuf, temp_buf, frame_size);
        int16_t *samples = (int16_t *)temp_buf;
        uint32_t num_samples = frame_size / 2; 
        last_sample[0] = samples[num_samples - 2]; 
        last_sample[1] = samples[num_samples - 1]; 
    } else {
        int16_t *samples = (int16_t *)temp_buf;
        for(int i = 0; i < frame_size / 2; i += 2) {
            samples[i] = last_sample[0];
            samples[i+1] = last_sample[1];
        }
    }

    memcpy(net_buf_add(buf, frame_size), temp_buf, frame_size);
    if (usb_audio_send((struct device *)dev, buf, frame_size) < 0) net_buf_unref(buf);
}

static const struct usb_audio_ops usb_ops = {
    .data_request_cb = data_request_cb, 
    .data_written_cb = data_written_cb,
    .data_received_cb = data_received_cb, 
    .feature_update_cb = feature_update_cb,
};

static void stream_started_cb(struct bt_bap_stream *stream) { LOG_INF("BAP Stream Started!"); }
static void stream_stopped_cb(struct bt_bap_stream *stream, uint8_t reason) { LOG_INF("BAP Stream Stopped"); }
static struct bt_bap_stream_ops stream_ops = { .started = stream_started_cb, .stopped = stream_stopped_cb, };

/* --- Main Audio Router Thread --- */
void audio_router_thread(void *p1, void *p2, void *p3)
{
    void *rx_mem_block;
    size_t rx_size;
    int16_t mono_temp[SAMPLES_PER_FRAME]; 

    k_sem_take(&hardware_ready_sem, K_FOREVER);
    
    while (1) {
        if (request_input_switch) {
            request_input_switch = false;
            is_line_active = !is_line_active;
            gpio_pin_set_dt(&led_line, is_line_active ? 1 : 0);  
            gpio_pin_set_dt(&led_mic, is_line_active ? 0 : 1);   
        }

        if (i2s_read(i2s_dev, &rx_mem_block, &rx_size) == 0) {
            if (rx_size == STEREO_BLOCK_SIZE) {
                if (is_line_active) {
                    int16_t *stereo_in = (int16_t *)rx_mem_block;
                    if (ring_buf_space_get(&audio_ringbuf) >= rx_size) {
                        ring_buf_put(&audio_ringbuf, (uint8_t *)stereo_in, rx_size);
                    }
                    for (int i = 0; i < SAMPLES_PER_FRAME; i++) {
                        int32_t sum = (stereo_in[i * 2] + stereo_in[i * 2 + 1]) / 2;
                        if (sum > 32767) sum = 32767;
                        else if (sum < -32768) sum = -32768;
                        mono_temp[i] = (int16_t)sum; 
                    }
                } else {
                    memset(mono_temp, 0, sizeof(mono_temp));
                }
            }
            k_mem_slab_free(&i2s_rx_slab, rx_mem_block);
        } else {
            memset(mono_temp, 0, sizeof(mono_temp));
            k_sleep(K_MSEC(5));
        }
        
        uint16_t current_seq = seq_num++;
        
        /* EXPERT FIX: Stereo Spoof (Encode Mono once, duplicate payload to 2 BIS streams) */
        if (streams[0].ep != NULL && streams[1].ep != NULL && lc3_encoder != NULL) {
            struct net_buf *buf_left = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
            struct net_buf *buf_right = net_buf_alloc(&bis_tx_pool, K_NO_WAIT);
            
            if (buf_left && buf_right) {
                net_buf_reserve(buf_left, BT_ISO_CHAN_SEND_RESERVE);
                net_buf_reserve(buf_right, BT_ISO_CHAN_SEND_RESERVE);
                
                uint8_t *enc_data = net_buf_add(buf_left, 100);
                lc3_encode(lc3_encoder, LC3_PCM_FORMAT_S16, mono_temp, 1, 100, enc_data);
                
                memcpy(net_buf_add(buf_right, 100), enc_data, 100);
                
                if (bt_bap_stream_send(&streams[0], buf_left, current_seq) < 0) {
                    net_buf_unref(buf_left);
                }
                if (bt_bap_stream_send(&streams[1], buf_right, current_seq) < 0) {
                    net_buf_unref(buf_right);
                }
            } else {
                if (buf_left) net_buf_unref(buf_left);
                if (buf_right) net_buf_unref(buf_right);
            }
        }
    }
}

K_THREAD_DEFINE(audio_router_id, 4096, audio_router_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(1), K_FP_REGS, 0);

/* --- Auracast Initialization --- */
void start_zip_auracast(void)
{
    int err;

    struct bt_le_adv_param ext_adv_param = {
        .id = BT_ID_DEFAULT,
        .options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_USE_IDENTITY,
        .interval_min = 0x00a0,
        .interval_max = 0x00f0,
    };

    err = bt_le_ext_adv_create(&ext_adv_param, NULL, &adv);
    
    /* EXPERT FIX: Locked PA interval to 80-100ms for strict Android discovery */
    struct bt_le_per_adv_param per_adv_param = {
        .interval_min = 0x0040, 
        .interval_max = 0x0050, 
        .options = 0,
    };
    err = bt_le_per_adv_set_param(adv, &per_adv_param);

    static const uint8_t custom_metadata[] = {
        0x03, BT_AUDIO_METADATA_TYPE_STREAM_CONTEXT, 0x04, 0x00,
        0x04, BT_AUDIO_METADATA_TYPE_LANG, 'e', 'n', 'g',
        13, BT_AUDIO_METADATA_TYPE_PROGRAM_INFO, 'Z', 'i', 'p', ' ', 'C', 'a', 'p', 't', 'i', 'o', 'n', 's'
    };
    
    memcpy(lc3_codec_cfg.meta, custom_metadata, sizeof(custom_metadata));
    lc3_codec_cfg.meta_len = sizeof(custom_metadata);

    /* EXPERT FIX: Declare 2 Streams in the subgroup */
    struct bt_bap_broadcast_source_stream_param stream_params[2] = {
        { .stream = &streams[0] },
        { .stream = &streams[1] }
    };
    struct bt_bap_broadcast_source_subgroup_param subgroup_param = {
        .params_count = 2, 
        .params = stream_params, 
        .codec_cfg = &lc3_codec_cfg,
    };
    struct bt_bap_broadcast_source_param create_param = {
        .params_count = 1, .params = &subgroup_param, .qos = &lc3_qos_cfg,
        .packing = BT_ISO_PACKING_SEQUENTIAL, .encryption = false,
    };

    err = bt_bap_broadcast_source_create(&create_param, &broadcast_source);
    active_bcast_id = sys_rand32_get() & 0x00FFFFFF;
    k_timer_start(&usb_pulse_timer, K_NO_WAIT, K_SECONDS(3));

    uint8_t pbp_svc_data[] = { 0x56, 0x18, 0x02, 0x00 };
    uint8_t bap_svc_data[] = {
        0x52, 0x18, 
        (uint8_t)(active_bcast_id & 0xFF),
        (uint8_t)((active_bcast_id >> 8) & 0xFF),
        (uint8_t)((active_bcast_id >> 16) & 0xFF)
    };

    static const uint8_t auracast_uuids[] = { 0x52, 0x18, 0x56, 0x18 };

    struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, "Zip Captions", 12),
        BT_DATA(BT_DATA_BROADCAST_NAME, "Zip Captions", 12),
        BT_DATA(BT_DATA_UUID16_ALL, auracast_uuids, sizeof(auracast_uuids)),
        BT_DATA(BT_DATA_SVC_DATA16, bap_svc_data, ARRAY_SIZE(bap_svc_data)),
        BT_DATA(BT_DATA_SVC_DATA16, pbp_svc_data, ARRAY_SIZE(pbp_svc_data)) 
    };

    err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);

    NET_BUF_SIMPLE_DEFINE(base_buf, 128);
    err = bt_bap_broadcast_source_get_base(broadcast_source, &base_buf);

    static uint8_t base_svc_data[128];
    base_svc_data[0] = BT_UUID_BASIC_AUDIO_VAL & 0xFF; 
    base_svc_data[1] = (BT_UUID_BASIC_AUDIO_VAL >> 8) & 0xFF;
    memcpy(&base_svc_data[2], base_buf.data, base_buf.len);

    struct bt_data per_ad[] = {
        BT_DATA(BT_DATA_SVC_DATA16, base_svc_data, base_buf.len + 2)
    };

    err = bt_le_per_adv_set_data(adv, per_ad, ARRAY_SIZE(per_ad));
    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    err = bt_le_per_adv_start(adv);
    err = bt_bap_broadcast_source_start(broadcast_source, adv);
}

/* --- System Initialization --- */
int main(void)
{
    lc3_encoder = lc3_setup_encoder(48000, 48000, 0, lc3_encoder_mem);
    
    NRF_CLOCK->TASKS_HFCLKAUDIOSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKAUDIOSTARTED == 0) {}
    
    /* 1. Wake Codec FIRST */
    wake_sgtl5000();

    /* 2. Configure I2S: nRF5340 acts as Master to generate MCLK for SGTL5000 */
    struct i2s_config i2s_cfg = {
        .word_size = 16, 
        .channels = 2,   
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER, 
        .frame_clk_freq = HW_SAMPLE_FREQ,
        .mem_slab = &i2s_rx_slab,
        .block_size = STEREO_BLOCK_SIZE, 
        .timeout = SYS_FOREVER_MS,
    };
    
    i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
    i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    
    /* 3. Bring up USB */
    mic_dev = DEVICE_DT_GET_ONE(usb_audio_mic);
    usb_audio_register((struct device *)mic_dev, &usb_ops);
    usb_enable(NULL);

    /* 4. Unblock Audio Thread */
    k_sem_give(&hardware_ready_sem);

    /* 5. Start Bluetooth */
    bt_enable(NULL);
    bt_bap_stream_cb_register(&streams[0], &stream_ops);
    bt_bap_stream_cb_register(&streams[1], &stream_ops);
    start_zip_auracast();

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}