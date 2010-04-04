#ifndef _PEEKNPOKE_INFO_H
#define _PEEKNPOKE_INFO_H

/*--------------------------------------------------------*/
/*-- Typedefs  --------------------------------------------*/
/*--------------------------------------------------------*/

/*--------------------------------------------------------*/
/*-- Defines   --------------------------------------------*/
/*--------------------------------------------------------*/
#
/*----------------------------------------------------------*/
/*-- PEEKNPOKE Registers and Structures      -------------------*/
/*-- Description:  Peek-And-Poke for registers -------------*/
/*-- Type:  Register access via ioctl()     ----------------*/
/*----------------------------------------------------------*/
#define DEV_PEEKNPOKE_BASE                  "/dev/peeknpoke"
#define DEV_PEEKNPOKE_MAJOR         	    120
#define MAX_PEEKNPOKE_MINORS                1
                                                                                                
typedef struct _peeknpokeregs 
{
        uint32_t offset;
        uint32_t value32;
        uint16_t value16;
        uint8_t value8;
} peeknpokeregs;
                                                                                                
#define PEEKNPOKE_GETREGS           _IOR(0x87, 0, peeknpokeregs)
#define PEEKNPOKE_SETREGS           _IOW(0x87, 1, peeknpokeregs)
#define PEEKNPOKE_GETREGS_16        _IOR(0x87, 2, peeknpokeregs)
#define PEEKNPOKE_SETREGS_16        _IOW(0x87, 3, peeknpokeregs)
#define PEEKNPOKE_GETREGS_8         _IOR(0x87, 4, peeknpokeregs)
#define PEEKNPOKE_SETREGS_8         _IOW(0x87, 5, peeknpokeregs)

#endif // _PEEKNPOKE_INFO_H
