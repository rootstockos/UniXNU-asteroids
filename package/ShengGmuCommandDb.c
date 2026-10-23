/* SPDX-License-Identifier: BSD-2-Clause-Patent */
#include "ShengGmuCommandDb.h"
#include "ShengGmuCommandDbSource.h"
static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|(uint16_t)p[1]<<8);}
static uint32_t u32(const uint8_t *p){return u16(p)|(uint32_t)u16(p+2)<<16;}
static int range(size_t at,size_t len,size_t size){return at<=size && len<=size-at;}
static int overlaps(size_t a,size_t n,size_t b,size_t m){return n && m && a<b+m && b<a+n;}
static int equal(const uint8_t *a,const char *b)
{for(unsigned i=0;i<8;++i)if(a[i]!=(uint8_t)b[i])return 0;return 1;}
ShengGmuDbStatus ShengGmuCommandDbInputs(const uint8_t *data,size_t bytes,uint32_t expected,ShengGmuVoteInputs *out)
{
    if(!data || !out || bytes!=SHENG_GMU_COMMAND_DB_BYTES || !expected)return ShengGmuDbArgument;
    if(u32(data)!=expected)return ShengGmuDbVersion;
    if(u32(data+4)!=0x0c0330db)return ShengGmuDbCorrupt;
    /* cmd_db_get_header: the slave table ends at the first zero slv_id, and every slave type is searched (int-011:
     * the sheng AOP database failed the earlier 3..5-only, no-gap rule). Only extents are checked for records that
     * are not used here; the records used below get the full address/slave/aux checks. */
    size_t heads[8]={0},sizes[8]={0},aux[8]={0};unsigned rows=0;
    for(unsigned i=0;i<8;++i) {
        const uint8_t *r=data+8+16*i;unsigned id=u16(r),count=u16(r+6);
        if(!id)break;
        heads[i]=144u+u16(r+2);sizes[i]=(size_t)count*24;aux[i]=144u+u16(r+4);
        if(!range(heads[i],sizes[i],bytes) || !range(aux[i],0,bytes))return ShengGmuDbCorrupt;
        ++rows;
    }
    static const char names[8][8]={{'g','f','x','.','l','v','l',0},{'m','x','.','l','v','l',0,0},
        {'c','x','.','l','v','l',0,0},{'g','m','x','c','.','l','v','l'},
        {'S','H','0',0,0,0,0,0},{'M','C','0',0,0,0,0,0},{'A','C','V',0,0,0,0,0},{'C','N','0',0,0,0,0,0}};
    ShengGmuAux found[8]={{0}};uint32_t addresses[8]={0};unsigned seen=0;
    for(unsigned i=0;i<rows;++i)for(size_t at=heads[i];at<heads[i]+sizes[i];at+=24) {
        const uint8_t *e=data+at;uint32_t address=u32(e+16);size_t len=u16(e+20),offset=aux[i]+u16(e+22);
        unsigned slave=u16(data+8+16*i);
        for(unsigned name=0;name<8;++name)if(equal(e,names[name])) {
            if(seen&(1u<<name))return ShengGmuDbDuplicate;
            if(!address || (address&3) || ((address>>16)&7)!=slave || !range(offset,len,bytes))return ShengGmuDbCorrupt;
            for(unsigned j=0;j<rows;++j)if(overlaps(offset,len,heads[j],sizes[j]))return ShengGmuDbCorrupt;
            if(slave!=(name<4?3u:5u))return ShengGmuDbCorrupt;
            seen|=1u<<name;found[name]=(ShengGmuAux){data+offset,len};addresses[name]=address;
        }
    }
    if((seen&0xf7)!=0xf7)return ShengGmuDbMissing;
    ShengGmuVoteInputs result=*out;
    result.gfx=found[0];result.mx=found[1];result.cx=found[2];result.gmxc=found[3];result.gmxc_present=(seen>>3)&1;
    result.sh0=(ShengGmuBcmInput){addresses[4],found[4]};result.mc0=(ShengGmuBcmInput){addresses[5],found[5]};
    result.acv=(ShengGmuBcmInput){addresses[6],found[6]};result.cn0_address=addresses[7];
    *out=result;return ShengGmuDbSuccess;
}
