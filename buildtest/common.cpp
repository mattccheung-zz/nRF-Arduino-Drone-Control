#include "common.h"
#include <TimerOne.h>

#include <SPI.h>

#include "cx10_nrf24l01.h"
#include "v202_nrf24l01.h"
#include "skeye_nrf24l01.h"
#include "bluefly_nrf24l01.h"
#include "x900_nrf24l01.h"


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
    #define PROTODEF(proto, module, map, cmd, name) case proto: PROTO_Cmds = cmd; break;
    switch(Model.protocol) {
        #include "protocol.h"
        default: PROTO_Cmds = NULL;
    }
    #undef PROTODEF

    //PROTOCOL_SetSwitch(get_module(Model.protocol));
}

