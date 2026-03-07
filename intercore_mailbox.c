#include "intercore_mailbox.h"

/* shared memory section */
#pragma section farbss "lmubss"
EndatCanMailbox g_endatCanMailbox = {0U, 0U, 0U};
#pragma section farbss restore
