/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "serialio.h"
#include "IfxAsclin_bf.h"

#ifdef __TASKING__
#include <io.h>
#endif

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void SERIALIO_Init (sint32 baudrate)
{
    /* Create an instance of the ASC handle */
    IfxAsclin_Asc ascHandle;

    /* Initialize an instance of IfxAsclin_Asc_Config with default values */
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, SERIALIO.asclin);

    ascConfig.baudrate.baudrate = (float32)baudrate;                            /* Set the desired baud rate         */

    /* Pin configuration */
    const IfxAsclin_Asc_Pins pins = {
        NULL_PTR, IfxPort_InputMode_pullUp,                                     /* CTS pin not used                  */
        SERIALIO.rx_pin, IfxPort_InputMode_pullUp,                              /* RX pin                            */
        NULL_PTR, IfxPort_OutputMode_pushPull,                                  /* RTS pin not used                  */
        SERIALIO.tx_pin, IfxPort_OutputMode_pushPull,                           /* TX pin                            */
        IfxPort_PadDriver_cmosAutomotiveSpeed1};
    ascConfig.pins = &pins;

    /* Initialize module with above parameters */
    IfxAsclin_Asc_initModule(&ascHandle, &ascConfig);

    /* Set the Transmit FIFO Level Flag (TFL) via the FLAGSSET register to start the transmission */
    SERIALIO.asclin->FLAGSSET.B.TFLS = 1;
}

#ifdef __TASKING__

/* Retarget for Tasking compiler */
void _io_putc (int c, struct _io *io)
{
    if (io->fp == NULL)
    {
        /* Called on print on string */
        /* If we still have enough space in the string */
        if (io->ptr < io->end)
        {
            *(io->ptr++) = (char)c;
        }
    }
    else
    {
        /* --- FIX: Insert CR before LF --- */
        if (c == '\n')
        {
             while (IfxAsclin_getTxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE);
             IfxAsclin_clearTxFifoFillLevelFlag(SERIALIO.asclin);
             IfxAsclin_writeTxData(SERIALIO.asclin, '\r');
        }
        /* -------------------------------- */

        while (IfxAsclin_getTxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE)
        {}

        IfxAsclin_clearTxFifoFillLevelFlag(SERIALIO.asclin);

        /* Send the character */
        IfxAsclin_writeTxData(SERIALIO.asclin, c);
    }
}

int _io_getc(struct _io *io)
{
    if (io->fp != NULL)
    {
        while (IfxAsclin_getRxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE)
        {}

        IfxAsclin_clearRxFifoFillLevelFlag(SERIALIO.asclin);

        /* Read the character */
        return IfxAsclin_readRxData(SERIALIO.asclin);
    }
    else
    {
        __debug();
    }
    return EOF;
}

#endif /* __TASKING__ */

#ifdef __HIGHTEC__

/* Retarget for GCC compiler */
int write (int desc, void *buf, size_t len)
{
    char *buf_char;
    int write_cnt;

    /* Descriptor is 1 for stdout */
    if (desc != 1)
    {
        __debug();
    }

    buf_char = (char *)buf;
    write_cnt = len;

    while (write_cnt)
    {
        /* --- FIX: Insert CR before LF --- */
        if (*buf_char == '\n')
        {
            while (IfxAsclin_getTxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE);
            IfxAsclin_clearTxFifoFillLevelFlag(SERIALIO.asclin);
            IfxAsclin_writeTxData(SERIALIO.asclin, '\r');
        }
        /* -------------------------------- */

        while (IfxAsclin_getTxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE)
        {}

        IfxAsclin_clearTxFifoFillLevelFlag(SERIALIO.asclin);

        /* Send the character */
        IfxAsclin_writeTxData(SERIALIO.asclin, *buf_char++);

        write_cnt -= 1;
    }
    return len;
}

size_t read (int __fd, void *__buf, size_t __nbyte)
{
    char single_buf_char = '\0';
    char *buf_char;
    const size_t buf_size = __nbyte;
    size_t read_cnt = 0;

    /* Descriptor is 0 for stdin */
    if (__fd != 0)
    {
        __debug();
    }

    buf_char = (char*) __buf;

    while(single_buf_char != '\r' && read_cnt < buf_size)
    {
        while (IfxAsclin_getRxFifoFillLevelFlagStatus(SERIALIO.asclin) != TRUE)
        {}

        IfxAsclin_clearRxFifoFillLevelFlag(SERIALIO.asclin);

        /* Read the character */
        single_buf_char = IfxAsclin_readRxData(SERIALIO.asclin);
        read_cnt++;

        /* Put the character in the buffer */
        *buf_char++ = single_buf_char;
    }

    return read_cnt;
}

#endif
