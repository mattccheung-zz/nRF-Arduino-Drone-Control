/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef MODULAR
  //Allows the linker to properly relocate
  #define V202_Cmds PROTO_Cmds
  #pragma long_calls
#endif
#include "common.h"
#include "interface.h"
#include "mixer.h"
//#include "config/model.h"
//#include "config/tx.h" // for Transmitter
#include "music.h"
//#include "telemetry.h"

#ifdef MODULAR
  //Some versions of gcc applythis to definitions, others to calls
  //So just use long_calls everywhere
  //#pragma long_calls_off
  extern unsigned _data_loadaddr;
  const unsigned long protocol_type = (unsigned long)&_data_loadaddr;
#endif

#ifdef PROTO_HAS_NRF24L01

#include "iface_nrf24l01.h"


#define BIND_COUNT 4

// Timeout for callback in uSec, 4ms=4000us for V202
#define PACKET_PERIOD 4000
// Time to wait for packet to be sent (no ACK, so very short)
#define PACKET_CHKTIME  100

// Every second
#define BLINK_COUNT 250
// ~ every 0.25 sec
#define BLINK_COUNT_MIN 64
// ~ every 2 sec
#define BLINK_COUNT_MAX 512


enum {
    // flags going to byte 14
    FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    FLAG_FLIP   = 0x04,
    FLAG_UNK9   = 0x08,
    FLAG_LED    = 0x10,
    FLAG_UNK10  = 0x20,
    FLAG_BIND   = 0xC0,
    // flags going to byte 10
    FLAG_HEADLESS  = 0x0200,
    FLAG_MAG_CAL_X = 0x0800,
    FLAG_MAG_CAL_Y = 0x2000
};

// For code readability
enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
    CHANNEL8,
    CHANNEL9,
    CHANNEL10,
    CHANNEL11
};

#define PAYLOADSIZE 10

static u8 packet[PAYLOADSIZE+1];
static u8 packet_sent;
static u8 tx_id[3];
static u8 rf_ch_num;
static u16 counter;
static u32 packet_counter;
static u8 tx_power;
//static u8 auto_flip; // Channel 6 <= 0 - disabled > 0 - enabled
static u16 led_blink_count;
static u8  throttle, rudder, elevator, aileron;
static u16 flags;


//
static u8 phase;
enum {
    V202_INIT2 = 0,
    V202_INIT2_NO_BIND =1,
    V202_BIND1=2,
    V202_BIND2=3,
    V202_DATA=4
};

//#define USE_BLINK_OPTION

static const char * const v202_opts[] = {
  _tr_noop("Re-bind"),  _tr_noop("No"), _tr_noop("Yes"), NULL,
  _tr_noop("250kbps"),  _tr_noop("No"), _tr_noop("Yes"), NULL,
#if defined(USE_BLINK_OPTION)
  _tr_noop("Blink"),  _tr_noop("No"), _tr_noop("Yes"), NULL,
#endif
  NULL
};
enum {
    PROTOOPTS_STARTBIND = 0,
    PROTOOPTS_BITRATE,
#if defined(USE_BLINK_OPTION)
    PROTOOPTS_USEBLINK,
#endif
    LAST_PROTO_OPT,
};
//ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

enum {
    STARTBIND_NO  = 0,
    STARTBIND_YES = 1,
};
enum {
    BITRATE_1MBPS   = 0,
    BITRATE_250KBPS = 1
};
#if defined(USE_BLINK_OPTION)
enum {
    USEBLINK_NO  = 0,
    USEBLINK_YES = 1,
};
#endif

// static u32 bind_count;

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id

static u8 freq1[2][8] = {{0x09,0x20,0x30,0x40},{0x11,0x21,0x31,0x41}};

static u8 rf_channels[4];

// Bit vector from bit position
#define BV(bit) (1 << bit)

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

static u8 packet_ack()
{
  return PKT_ACKED;
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT))) {
    case BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}

static void tx_init()
{
    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    
    NRF24L01_WriteReg(NRF24L01_09_CD, 0x00); 
    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x77); 
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x20);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x10);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x00);
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x4e);
    NRF24L01_ReadReg(0x1D);
    NRF24L01_Activate(0x73);
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_Activate(0x53); // magic for BK2421 bank switch
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_Activate(0x53);
    NRF24L01_ReadReg(NRF24L01_00_CONFIG);

    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8 *) "\xAB\xAC\xAD\xAE\xAF", 5);
}


static void set_tx_id(u32 id)
{
    u8 sum;
    tx_id[0] = 0x7b; //(id >> 16) & 0xFF;
    tx_id[1] = 0xe4; // (id >> 8) & 0xFF;
    tx_id[2] = 0xc1; // (id >> 0) & 0xFF;
    sum = tx_id[0] + tx_id[1] + tx_id[2];
    // Base row is defined by lowest 2 bits
    /*const u8 *fh_row = freq_hopping[sum & 0x03];
    // Higher 3 bits define increment to corresponding row
    u8 increment = (sum & 0x1e) >> 2;
    for (u8 i = 0; i < 8; ++i) {
        u8 val = fh_row[i] + increment;
        // Strange avoidance of channels divisible by 16
        rf_channels[i] = (val & 0x0f) ? val : val - 3;
    }*/
}

static u8 pkt_checksum()
{
  u8 sum = 0;
  for (u8 i = 0; i < 9;  ++i) sum ^= packet[i];
  return sum + 0x55;
}


static u8 convert_channel(u8 num)
{
    s32 ch = Channels[num];
    if (ch < CHAN_MIN_VALUE) {
        ch = CHAN_MIN_VALUE;
    } else if (ch > CHAN_MAX_VALUE) {
        ch = CHAN_MAX_VALUE;
    }
    return (u8) (((ch * 0xFF / CHAN_MAX_VALUE) + 0x100) >> 1);
}


static void read_controls(u8* throttle, u8* rudder, u8* elevator, u8* aileron,
                          u16* flags, u16* led_blink_count)
{
    // Protocol is registered AETRG, that is
    // Aileron is channel 0, Elevator - 1, Throttle - 2, Rudder - 3
    // Sometimes due to imperfect calibration or mixer settings
    // throttle can be less than CHAN_MIN_VALUE or larger than
    // CHAN_MAX_VALUE. As we have no space here, we hard-limit
    // channels values by min..max range
    u8 a;

    // Channel 3
    *throttle = Channels[CHANNEL3]; //convert_channel(CHANNEL3);

    // Channel 4
    //a = convert_channel(CHANNEL4);
    *rudder = Channels[CHANNEL4]; //a < 0x80 ? 0x7f - a : a;

    // Channel 2
    //a = convert_channel(CHANNEL2);
    *elevator = Channels[CHANNEL2]; //a < 0x80 ? 0x7f - a : a;

    // Channel 1
    //a = convert_channel(CHANNEL1);
    *aileron = Channels[CHANNEL1]; //a < 0x80 ? 0x7f - a : a;

    // Channel 5
    // 512 - slow blinking (period 4ms*2*512 ~ 4sec), 64 - fast blinking (4ms*2*64 ~ 0.5sec)
    

    int num_channels = Model.num_channels;


    // Channel 6
    if (Channels[CHANNEL6] <= 0) *flags &= ~FLAG_FLIP;
    else *flags |= FLAG_FLIP;

    // Channel 7
    if (num_channels < 7 || Channels[CHANNEL7] <= 0) *flags &= ~FLAG_CAMERA;
    else *flags |= FLAG_CAMERA;

    // Channel 8
    if (num_channels < 8 || Channels[CHANNEL8] <= 0) *flags &= ~FLAG_VIDEO;
    else *flags |= FLAG_VIDEO;

    // Channel 9
    if (num_channels < 9 || Channels[CHANNEL9] <= 0) *flags &= ~FLAG_HEADLESS;
    else *flags |= FLAG_HEADLESS;

    // Channel 10
    if (num_channels < 10 || Channels[CHANNEL10] <= 0) *flags &= ~FLAG_MAG_CAL_X;
    else *flags |= FLAG_MAG_CAL_X;

    // Channel 10
    if (num_channels < 11 || Channels[CHANNEL11] <= 0) *flags &= ~FLAG_MAG_CAL_Y;
    else *flags |= FLAG_MAG_CAL_Y;

    // Print channels every second or so
    /*if ((packet_counter & 0xFF) == 1) {
        printf("Raw channels: %d, %d, %d, %d, %d, %d, %d, %d\n",
               Channels[0], Channels[1], Channels[2], Channels[3],
               Channels[4], Channels[5], Channels[6], Channels[7]);
        printf("Aileron %d, elevator %d, throttle %d, rudder %d, led_blink_count %d\n",
               (s16) *aileron, (s16) *elevator, (s16) *throttle, (s16) *rudder,
               (s16) *led_blink_count);
    }*/
}

static void send_packet(u8 bind)
{
  
    switch (bind) {
        case 1:
          packet[0] = 0xA1;
          packet[1] = 0xF4;
          packet[2] = 0x30;
          packet[3] = 0x57;
          packet[4] = 0x47;
          packet[5] = 0xAA;
          packet[6] = 0xAA;
          packet[7] = 0xBB;
          packet[8] = 0xB1;
          packet[9] = 0xD4;
          break;
        case 2:
          packet[0] = 0x0A;
          packet[1] = 0x1A;
          packet[2] = 0x2A;
          packet[3] = 0x3A;
          packet[4] = 0x12;
          packet[5] = 0x22;
          packet[6] = 0x32;
          packet[7] = 0x42;
          packet[8] = 0xB1;
          packet[9] = 0x46;
          break;
        case 0:          
          read_controls(&throttle, &rudder, &elevator, &aileron, &flags, &led_blink_count);
          packet[0] = throttle;
          packet[1] = rudder;
          packet[2] = elevator;
          packet[3] = aileron;
          
          packet[4] = 0x00;
          packet[5] = 0x45; 
          packet[6] = 0x00;
          packet[7] = 0x21;
          packet[8] = 0x00;
          break;
    }
    packet[9] = pkt_checksum();

    packet_sent = 0;
    // Each packet is repeated twice on the same
    // channel, hence >> 1
    // We're not strictly repeating them, rather we
    // send new packet on the same frequency, so the
    // receiver gets the freshest command. As receiver
    // hops to a new frequency as soon as valid packet
    // received it does not matter that the packet is
    // not the same one repeated twice - nobody checks this
    u8 rf_ch = rf_channels[rf_ch_num >> 1];
    rf_ch_num = (rf_ch_num + 1) & 0xF;
    //  Serial.print(rf_ch); Serial.write("\n");
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x2e);
    NRF24L01_FlushTx();
    NRF24L01_ReadReg(NRF24L01_00_CONFIG);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
 
    NRF24L01_WritePayload(packet, PAYLOADSIZE);
    ++packet_counter;
    packet_sent = 1;

    if ((packet_counter & 0xFF) == 1) {
     for (int i =0; i< sizeof(packet); i++) {Serial.print(packet[i], HEX); Serial.print("-");  }  Serial.println();
    }

    // Check and adjust transmission power. We do this after
    // transmission to not bother with timeout after power
    // settings change -  we have plenty of time until next
    // packet.
    /*if (! rf_ch_num && tx_power != Model.tx_power) {
        //Keep transmit power updated
        tx_power = Model.tx_power;
        NRF24L01_SetPower(tx_power);
    }*/
}

int bindcnt = -1;
MODULE_CALLTYPE
static u16 skeye_callback()
{
    switch (phase) {
    case V202_INIT2:
        //V202_init();
        MUSIC_Play(MUSIC_TELEMALARM1);
        for(int i=0; i<4; i++) {
          rf_channels[i] = freq1[0][i];
        }
        phase = V202_BIND1;
        PROTOCOL_SetBindState(0);
        break;
        
    case V202_BIND1:
        // first bind sequence, we repeat it two times
        bindcnt++;
        switch (bindcnt >> 3) {
          case 0:
            send_packet(1);
            return PACKET_PERIOD;
            break;
          case 1:
            send_packet(2);
            break;
          case 2:
            bindcnt = -1;
            if (--counter == 0) {
              for(int i=0; i<4; i++) {
                rf_channels[i] = freq1[1][i];
              }
              NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x77); 
              NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x27);
              NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8 *) "\x47\x57\x30\xF4\xA1", 5);
              phase = V202_BIND2;
              counter = BIND_COUNT;
            } 
            break;
        }
        break;
    case V202_BIND2:
        if (bindcnt++ < 8)
          send_packet(1);
        else if (bindcnt < 16)
          send_packet(2);
        else 
        if (--counter == 0) {
            phase = V202_DATA;
            counter = led_blink_count;
            bindcnt = -1;
            flags = 0;
            
            MUSIC_Play(MUSIC_DONE_BINDING);
        } else {
          bindcnt = -1;
        }
        break;
    case V202_DATA:
        
        
        send_packet(0);
        break;
    }
    // Packet every 4ms
    return PACKET_PERIOD;
}

// Generate internal id from TX id and manufacturer id (STM32 unique id)
static void initialize_tx_id()
{
    u32 lfsr = 0xb2c54a2ful;

#ifndef USE_FIXED_MFGID
    u8 var[12];
    MCU_SerialNumber(var, 12);
    Serial.print("Manufacturer id: ");
    for (int i = 0; i < 12; ++i) {
        Serial.print(var[i], HEX); Serial.print(" - ");
        rand32_r(&lfsr, var[i]);
    }
    Serial.println("");
#endif

    if (Model.fixed_id) {
       for (u8 i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8)
           rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    }
    // Pump zero bytes for LFSR to diverge more
    for (u8 i = 0; i < sizeof(lfsr); ++i) rand32_r(&lfsr, 0);

    set_tx_id(lfsr);
}

static void initialize(u8 bind)
{
    CLOCK_StopTimer();
    tx_power = Model.tx_power;
    packet_counter = 0;
    led_blink_count = BLINK_COUNT_MAX;
    tx_init();
    phase = V202_INIT2;
    
    counter = BIND_COUNT;
    //PROTOCOL_SetBindState(BIND_COUNT * PACKET_PERIOD / 1000); //msec


    initialize_tx_id();

    CLOCK_StartTimer(PACKET_PERIOD*2, skeye_callback);
}

const void *Skeye_Cmds(enum ProtoCmds cmd)
{
    switch(cmd) {
        case PROTOCMD_INIT:
        
            initialize(0);
            return 0;
        case PROTOCMD_DEINIT:
        case PROTOCMD_RESET:
            CLOCK_StopTimer();
            return (void *)(NRF24L01_Reset() ? 1L : -1L);
        case PROTOCMD_CHECK_AUTOBIND: return (void *)0L; //Never Autobind
        case PROTOCMD_BIND: initialize(1); return 0;
        case PROTOCMD_NUMCHAN: return (void *) 11L; // T, R, E, A, LED (on/off/blink), Auto flip, camera, video, headless, X-Y calibration
        case PROTOCMD_DEFAULT_NUMCHAN: return (void *)6L;
        // TODO: return id correctly
        case PROTOCMD_CURRENT_ID: return Model.fixed_id ? (void *)((unsigned long)Model.fixed_id) : 0;
        case PROTOCMD_GETOPTIONS: return v202_opts;
        case PROTOCMD_TELEMETRYSTATE: return (void *)(long)PROTO_TELEM_UNSUPPORTED;
        default: break;
    }
    return 0;
}
#endif
