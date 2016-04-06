 #include "Arduino.h"
 #include"interface.h"

#define PROTO_HAS_NRF24L01 true
//#define PROTO_HAS_CC2500 true
#define PROTO_HAS_X true

#define MODULE_CALLTYPE

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
//typedef uint16_t u16;
//typedef uint32_t u32;
typedef uint64_t u64;

#define PROTO_MAP_LEN 5
#define PROTOCOL_COUNT 30

#define NUM_PROTO_OPTS 4
#define NUM_CHANNELS 32
#define NUM_OUT_CHANNELS 32
extern volatile s32 Channels[NUM_OUT_CHANNELS];

#define CHAN_MULTIPLIER 10
#define PCT_TO_RANGE(x) ((x) * CHAN_MULTIPLIER)
#define RANGE_TO_PCT(x) ((x) / CHAN_MULTIPLIER)
#define CHAN_MAX_VALUE (100 * CHAN_MULTIPLIER)
#define CHAN_MIN_VALUE (-100 * CHAN_MULTIPLIER)

#ifndef ctassert
#define ctassert(x, y)
static void workarround() { return; };
#endif

#define _tr_noop(x) x
#define _tr(x) x

enum {
  INP_ELEVATOR = 0, INP_AILERON, INP_THROTTLE, INP_RUDDER, INP_GEAR1 
};

enum {
    PROTO_TELEM_UNSUPPORTED = -1,
    PROTO_TELEM_OFF = 0,
    PROTO_TELEM_ON  = 1,
};

enum TxPower {
    TXPOWER_100uW = 0,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};

#define PROTODEF(proto, module, map, init, name) proto,
enum Protocols {
    PROTOCOL_NONE = 0,
    #include "protocol.h"
};
#undef PROTODEF

//extern const u8 *ProtocolChannelMap[PROTOCOL_COUNT];
//extern const char * const ProtocolNames[PROTOCOL_COUNT];*/

const u8 EATRG[PROTO_MAP_LEN] =
    { INP_ELEVATOR, INP_AILERON, INP_THROTTLE, INP_RUDDER, INP_GEAR1 };
static const u8 TAERG[PROTO_MAP_LEN] = 
    { INP_THROTTLE, INP_AILERON, INP_ELEVATOR, INP_RUDDER, INP_GEAR1 };
static const u8 AETRG[PROTO_MAP_LEN] = 
    { INP_AILERON, INP_ELEVATOR, INP_THROTTLE, INP_RUDDER, INP_GEAR1 };
    
#define PROTODEF(proto, module, map, cmd, name) name,
const char * const ProtocolNames[PROTOCOL_COUNT] = {
    "None",
    #include "protocol.h"
};
#undef PROTODEF


extern const u8 *ProtocolChannelMap[PROTOCOL_COUNT];

#define PROTO_MAP_LEN 5

enum ModelType {
    MODELTYPE_HELI,
    MODELTYPE_PLANE,
    MODELTYPE_MULTI,
};

/*void PROTOCOL_Init(u8 force);
void PROTOCOL_Load(int no_dlg);
void PROTOCOL_DeInit();
u8 PROTOCOL_WaitingForSafe();
u64 PROTOCOL_CheckSafe();
u32 PROTOCOL_Binding();
u8 PROTOCOL_AutoBindEnabled();
void PROTOCOL_Bind();
void PROTOCOL_SetBindState(u32 msec);
int PROTOCOL_NumChannels();
u8 PROTOCOL_GetTelemCapability();
int PROTOCOL_DefaultNumChannels();
void PROTOCOL_CheckDialogs();
u32 PROTOCOL_CurrentID();
const char **PROTOCOL_GetOptions();
void PROTOCOL_SetOptions();
int PROTOCOL_GetTelemetryState();
int PROTOCOL_MapChannel(int input, int default_ch);
int PROTOCOL_HasModule(int idx);
int PROTOCOL_HasPowerAmp(int idx);
int PROTOCOL_SetSwitch(int module);
int PROTOCOL_SticksMoved(int init);
void PROTOCOL_InitModules();*/

struct ModelStruct {
    u32 fixed_id;
    enum ModelType type;
    enum Protocols protocol;
    s16 proto_opts[NUM_PROTO_OPTS];
    u8 num_channels;
    u8 num_ppmin;
    u16 ppmin_centerpw;
    u16 ppmin_deltapw;
    u8 train_sw;
    enum TxPower tx_power;
//    enum SwashType swash_type;
    u8 swash_invert;
    u8 swashmix[3];
    char name[24];
    char icon[24];
//    char virtname[NUM_VIRT_CHANNELS][VIRT_NAME_LEN];
    u8 templates[NUM_CHANNELS];
//    u8 safety[NUM_SOURCES+1];
 //   u8 telem_alarm[TELEM_NUM_ALARMS];
//    s32 telem_alarm_val[TELEM_NUM_ALARMS];
    u8 telem_flags;
//    MixerMode mixer_mode;
//    s8 ppm_map[MAX_PPM_IN_CHANNELS];
    u8 padding_1[2];
    u32 permanent_timer;
#if HAS_VIDEO
    u8 videosrc;
    u8 videoch;
    s8 video_contrast;
    s8 video_brightness;
#endif
//    struct Trim trims[NUM_TRIMS];
//    struct Mixer mixers[NUM_MIXERS];
//    struct Limit limits[NUM_OUT_CHANNELS];
//    struct Timer timer[NUM_TIMERS];
//    struct PageCfg2 pagecfg2;
#if HAS_DATALOG
    struct datalog datalog;
#endif
};

u32 rand32_r(u32 *seed, u8 update);

extern struct ModelStruct Model;



enum Music {
    MUSIC_STARTUP = 0,
    MUSIC_SHUTDOWN,
    MUSIC_VOLUME,
    MUSIC_ALARM1,
    MUSIC_ALARM2,
    MUSIC_ALARM3,
    MUSIC_ALARM4,
    MUSIC_BATT_ALARM,
    MUSIC_DONE_BINDING,
    MUSIC_TIMER_WARNING,
    MUSIC_KEY_PRESSING,
    MUSIC_SAVING,
    MUSIC_MAXLEN,
    MUSIC_TELEMALARM1,
    MUSIC_TELEMALARM2,
    MUSIC_TELEMALARM3,
    MUSIC_TELEMALARM4,
    MUSIC_TELEMALARM5,
    MUSIC_TELEMALARM6,
};

void MUSIC_Play(enum Music music);

void PROTOCOL_SetBindState(u32 msec);

void MCU_SerialNumber(u8 *var, int len);

void CLOCK_StartTimer(unsigned us, u16 (*cb)(void));
void CLOCK_StopTimer();

void PROTOCOL_Load(int no_dlg);
void PROTOCOL_Run(enum ProtoCmds cmds);

u32 Crc(const void *buffer, u32 size);

extern void * (*PROTO_Cmds)(enum ProtoCmds);
#define PROTOCOL_LOADED PROTO_Cmds

struct mcu_pin {
    u32 port;
    u32 pin; //This only needs to be u16, but we need the struct to be word-aligned
};

enum {
    CYRF6936,
    A7105,
    CC2500,
    NRF24L01,
    MULTIMOD,
    TX_MODULE_LAST,
};

#define TX_MODULE_LAST 20

extern struct mcu_pin MODULE_ENABLE[TX_MODULE_LAST];

uint16_t spi_xfer(uint32_t spi, uint16_t data);

#define SPI2 0
#define PROTOSPI_pin_set(io) digitalWrite(io,HIGH)
#define PROTOSPI_pin_clear(io) digitalWrite(io,LOW)
#define PROTOSPI_xfer(byte) spi_xfer(SPI2, byte)
#define _NOP()  asm volatile ("nop")

#define usleep(x) delayMicroseconds(x)


struct Transmitter {
    u8 current_model;
    u8 language;
    u8 backlight;
    u8 contrast;
    u8 telem;
    u8 music_shutdown;
    u8 extra_hardware;
 //   enum Mode mode;
    u16 batt_alarm;
    u16 batt_critical;
    u16 batt_warning_interval;
    u8 power_alarm;
    u8 splash_delay;
    u8 volume;
    u8 module_poweramp;
    u8 vibration_state; // for future vibration on/off support
    u8 padding_1[1];
#if HAS_RTC
    u8 rtcflags;    // bit0: clock12hr, bit1-3: time format, bit4-7 date format (see pages/320x240x16/rtc_config.c)
#endif
    
    #ifdef HAS_MORE_THAN_32_INPUTS
        u64 ignore_src;
    #else
        u32 ignore_src;
    #endif

    struct mcu_pin module_enable[TX_MODULE_LAST];
    u32 txid;
    //struct StickCalibration calibration[INP_HAS_CALIBRATION];
    //struct TouchCalibration touch;
    //struct AutoDimmer auto_dimmer;
    //struct CountDownTimerSettings countdown_timer_settings;
};

extern struct Transmitter Transmitter;

