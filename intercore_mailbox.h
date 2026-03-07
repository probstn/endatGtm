#ifndef INTERCORE_MAILBOX_H_
#define INTERCORE_MAILBOX_H_

#include "Ifx_Types.h"

/* One-way mailbox: CPU0 -> CPU2 */
typedef struct
{
    volatile uint32 seq;
    volatile uint32 pos20;
    volatile uint32 valid;
} EndatCanMailbox;

/* Nur Deklaration, keine Definition */
extern EndatCanMailbox g_endatCanMailbox;

#endif
