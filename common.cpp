#include "common.h"
#include <TimerOne.h>

#include <SPI.h>

/*#include "skeye_nrf24l01.h"
#include "x900_nrf24l01.h"
#include "bluefly_nrf24l01.h" 
#include "cflie_nrf24l01.h"
#include "cg023_nrf24l01.h"   
#include "cx10_nrf24l01.h"    
#include "devo_cyrf6936.h"    
#include "dsm2_cyrf6936.h"    
#include "esky150_nrf24l01.h" 
#include "esky_nrf24l01.h"    
#include "flysky_a7105.h"     
#include "frsky1way_cc2500.h" 
#include "frsky2way_cc2500.h" 
#include "h377_nrf24l01.h"  
#include "hisky_nrf24l01.h"  
#include "hm830_nrf24l01.h"  
#include "hontai_nrf24l01.h"  
#include "hubsan_a7105.h"     
#include "iface_nrf24l01.h"    
#include "j6pro_cyrf6936.h"   
#include "joysway_a7105.h"    
#include "kn_nrf24l01.h"  
#include "ne260_nrf24l01.h"
#include "skeye_nrf24l01.h" 
#include "skyartec_cc2500.h"
#include "slt_nrf24l01.h"  */
#include "symax_nrf24l01.h"
#include "v202_nrf24l01.h"
//#include "wk2x01.h"           
//#include "x900_nrf24l01.h"    
#include "yd717_nrf24l01.h"
//#include "gp993_nrf24l01.h"
//#include "yd828_nrf24l01.h"

#define PROTODEF(proto, module, map, cmd, name) map,
const u8 *ProtocolChannelMap[PROTOCOL_COUNT] = {
    NULL,
    #include "protocol.h"
};
#undef PROTODEF


void *PPMOUT_Cmds(enum ProtoCmds cmd) {
  
}

void *USBHID_Cmds(enum ProtoCmds cmd) {
  
}

void *TEST_Cmds(enum ProtoCmds cmd) {
  
}

//The folloiwng code came from: http://notabs.org/winzipcrc/winzipcrc.c
// C99 winzip crc function, by Scott Duplichan
//We could use the internal CRC implementation in the STM32, but this is really small
//and perfomrance isn't really an issue
u32 Crc(const void *buffer, u32 size)
{
   u32 crc = ~0;
   const u8  *position = (const u8  *)buffer;

   while (size--) 
      {
      int bit;
      crc ^= *position++;
      for (bit = 0; bit < 8; bit++) 
         {
         s32 out = crc & 1;
         crc >>= 1;
         crc ^= -out & 0xEDB88320;
         }
      }
   return ~crc;
}

static u32 rand_seed = 0xb2c54a2ful;
// Linear feedback shift register with 32-bit Xilinx polinomial x^32 + x^22 + x^2 + x + 1
static const uint32_t LFSR_FEEDBACK = 0x80200003ul;
static const uint32_t LFSR_INTAP = 32-1;
static void update_lfsr(uint32_t *lfsr, uint8_t b)
{
    for (int i = 0; i < 8; ++i) {
        *lfsr = (*lfsr >> 1) ^ ((-(*lfsr & 1u) & LFSR_FEEDBACK) ^ ~((uint32_t)(b & 1) << LFSR_INTAP));
        b >>= 1;
    }
}

u32 rand32_r(u32 *seed, u8 update)
{
    if(! seed)
        seed = &rand_seed;
    update_lfsr(seed, update);
    return *seed;
}

void MUSIC_Play(enum Music music) {
  // NOTHING
}

void PROTOCOL_SetBindState(u32 msec) {
  // NOTHING?
}

void MCU_SerialNumber(u8 *var, int len) {
  // NOTHING?
}

u16 (*timer_callback)(void);


void timer1isr() {
  //Timer1.stop();
  if(timer_callback) {
        u16 us = timer_callback();
        //timer_clear_flag(TIM5, TIM_SR_CC1IF);
        if (us) {
            Timer1.setPeriod(us);
            //Timer1.start();
            //timer_set_oc_value(TIM5, TIM_OC1, us + TIM_CCR1(TIM5));
            return;
        }
    }
    //CLOCK_StopTimer();
  
}

void CLOCK_StartTimer(unsigned us, u16 (*cb)(void)){
  timer_callback = cb;
  Timer1.disablePwm(9);
  Timer1.disablePwm(10);
  Timer1.initialize(us);
  Timer1.attachInterrupt(timer1isr);
  //Timer1.start();
  /*noInterrupts();
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  interrupts();*/
}

void CLOCK_StopTimer() {
  //TIMSK0 &= _BV(1 << OCIE0A);
  Timer1.stop();
  Timer1.detachInterrupt();
  timer_callback = NULL;
}


uint16_t spi_xfer(uint32_t spi, uint16_t data) {
  SPI.transfer(data);
  //SPI.send(data);
}

void PROTOCOL_Load(int no_dlg)
{
    /*if(! PROTOCOL_HasModule(Model.protocol)) {
        PROTO_Cmds = NULL;
        printf("Module is not defined!\n");
        return;
    }*/
    #define PROTODEF(proto, module, map, cmd, name) case proto: PROTO_Cmds = cmd; Serial.print(name); Serial.println(" -- active protocol"); break;
    switch(no_dlg) {
        #include "protocol.h"
        default: Serial.println("no protocol found"); PROTO_Cmds = NULL;
    }
    #undef PROTODEF

    //PROTOCOL_SetSwitch(get_module(Model.protocol));
}

void PROTOCOL_Run(enum ProtoCmds cmds) {
  if (PROTO_Cmds == NULL) return;
  PROTO_Cmds(cmds);
}

