#include "zimani_control.h"
static uint32_t u32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static int16_t i16(const uint8_t *p)
{ uint16_t x=(uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); return (int16_t)x; }
uint16_t z_crc16(const uint8_t *p,uint32_t n)
{
    uint16_t crc=0xFFFFu;
    while(n--) { crc^=(uint16_t)((uint16_t)*p++<<8);
        for(unsigned i=0;i<8;i++) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:((uint32_t)crc<<1)); }
    return crc;
}
void z_stop(ZControl *c,uint8_t fault)
{
    c->armed=0;c->can_arm=0;c->fault=fault;
    c->command.forward=0;c->command.turn=0;c->command.blade=0;
    c->command.cruise=0;c->command.light=0;
}
void z_init(ZControl *c)
{
    *c=(ZControl){0};c->command.p9=DEFAULT_P9;z_stop(c,FAULT_BOOT);
}
void z_link(ZControl *c,uint32_t now) {c->link_seen=1;c->last_link_us=now;}
void z_tick(ZControl *c,uint32_t now)
{
    if(c->host_seen && (uint32_t)(now-c->last_host_us)>=HOST_TIMEOUT_US) {
        z_stop(c,FAULT_HOST);c->host_seen=0;
    }
    if(c->link_seen && (uint32_t)(now-c->last_link_us)>=LINK_TIMEOUT_US) {
        c->link_seen=0;
        if(c->armed) z_stop(c,FAULT_LINK);
    }
}
int z_accept(ZControl *c,const uint8_t *f,uint32_t now)
{
    uint16_t crc=z_crc16(f,20u);
    uint32_t session=u32(f+4),seq=u32(f+8);
    int16_t forward=i16(f+12),turn=i16(f+14);
    uint8_t blade=f[16],bits=f[17],p9=f[18];
    if(f[0]!=0xC3u||f[1]!=0x3Cu||f[2]!=1u||f[3]!=1u
       ||f[20]!=(uint8_t)crc||f[21]!=(uint8_t)(crc>>8)||!session
       ||forward < -255||forward>255||turn < -255||turn>255
       ||blade>2u||(bits&~7u)||f[19]||(p9!=0u&&p9!=2u)) goto reject;
    z_tick(c,now);
    if(session!=c->session) {
        if(bits&HOST_ARM) goto reject;
        z_stop(c,0);c->session=session;c->seq=seq;
    } else {
        uint32_t delta=seq-c->seq;
        if(!delta || delta>=0x80000000u) goto reject;
        c->seq=seq;
    }
    c->last_host_us=now;c->host_seen=1;c->accepted++;
    if(!(bits&HOST_ARM)) {
        z_stop(c,0);c->command.p9=p9;c->can_arm=1;return 1;
    }
    if(!c->armed) {
        if(!c->can_arm||!c->link_seen||forward||turn||blade||(bits&HOST_CRUISE)) {
            z_stop(c,c->link_seen?FAULT_ARM:FAULT_LINK);return 1;
        }
        c->armed=1;c->can_arm=0;
    }
    if(p9!=c->command.p9) {z_stop(c,FAULT_ARM);return 1;}
    c->fault=0;c->command.forward=forward;c->command.turn=turn;
    c->command.blade=blade;c->command.cruise=(bits&HOST_CRUISE)?1u:0u;
    c->command.light=(bits&HOST_LIGHT)?1u:0u;
    return 1;
reject:
    c->rejected++;return 0;
}
void z_payload(const ZCommand *c,uint8_t p[26])
{
    static const uint8_t header[]={ZIMANI_HEADER_BYTES};
    for(unsigned i=0;i<26;i++) p[i]=0;
    for(unsigned i=0;i<9;i++) p[i]=header[i];
    p[9]=c->p9;
    if(c->forward>0) p[11]|=1u;
    if(c->forward<0) p[11]|=2u;
    if(c->turn<0) p[11]|=4u;
    if(c->turn>0) p[11]|=8u;
    p[12]=(uint8_t)(c->forward<0?-c->forward:c->forward);
    p[13]=(uint8_t)(c->turn<0?-c->turn:c->turn);
    p[14]=100;p[15]=c->blade;p[17]=c->light;p[18]=c->cruise;p[21]=1;
    for(unsigned i=0;i<24;i++) p[24]=(uint8_t)(p[24]+p[i]);
    p[25]=0xED;
}
int z_reply_valid(const uint8_t *p,uint32_t size)
{
    static const uint8_t header[]={ZIMANI_HEADER_BYTES};
    if(size!=23u||p[0]!=0xBBu||p[22]!=0xEDu) return 0;
    for(unsigned i=1;i<9;i++) if(p[i]!=header[i]) return 0;
    uint8_t s=0;for(unsigned i=0;i<21;i++) s=(uint8_t)(s+p[i]);
    return s==p[21];
}
