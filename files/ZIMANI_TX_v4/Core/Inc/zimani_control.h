#ifndef ZIMANI_CONTROL_H
#define ZIMANI_CONTROL_H
#include <stdint.h>
#include "zimani_config.h"
#define HOST_FRAME_SIZE 22u
#define HOST_ARM 1u
#define HOST_CRUISE 2u
#define HOST_LIGHT 4u
#define HOST_ESTOP 8u
#define HOST_RESET_STOP 16u
enum { FAULT_BOOT=1,FAULT_HOST=2,FAULT_LINK=3,FAULT_RADIO=4,FAULT_ARM=5,FAULT_UART=6,FAULT_BUTTON=7,FAULT_STOP=8 };
typedef struct {
    int16_t forward,turn;
    uint8_t blade,cruise,light,engine,engine_adjust,estop;
} ZCommand;
typedef struct {
    uint32_t session,seq,last_host_us,last_link_us,accepted,rejected;
    uint8_t host_seen,link_seen,armed,can_arm,fault;
    ZCommand command;
} ZControl;
uint16_t z_crc16(const uint8_t *p,uint32_t n);
void z_init(ZControl *c);
void z_stop(ZControl *c,uint8_t fault);
void z_link(ZControl *c,uint32_t now);
void z_tick(ZControl *c,uint32_t now);
int z_accept(ZControl *c,const uint8_t *frame,uint32_t now);
void z_payload(const ZCommand *cmd,uint8_t out[26]);
int z_reply_valid(const uint8_t *p,uint32_t size);
#endif
