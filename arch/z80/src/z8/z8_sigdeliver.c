/****************************************************************************
 * arch/z80/src/z8/z8_sigdeliver.c
 *
 *   Copyright (C) 2008-2010, 2015, 2018-2019 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "chip/switch.h"
#include "sched/sched.h"
#include "up_internal.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_copystate
 ****************************************************************************/

/* Maybe a little faster than most memcpy's */

static void z8_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src)
{
  int i;
  for (i = 0; i < XCPTCONTEXT_REGS; i++)
    {
      *dest++ = *src++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
#ifndef CONFIG_DISABLE_SIGNALS
  FAR struct tcb_s *rtcb = this_task();
  chipreg_t regs[XCPTCONTEXT_REGS];

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = rtcb->pterrno;

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  DEBUGASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the return state on the stack. */

  z8_copystate(regs, rtcb->xcp.regs);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signals */

  ((sig_deliver_t)rtcb->xcp.sigdeliver)(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   */

  sinfo("Resuming\n");
  (void)up_irq_save();
  rtcb->pterrno        = saved_errno;

  /* Modify the saved return state with the actual saved values in the
   * TCB.  This depends on the fact that nested signal handling is
   * not supported.  Therefore, these values will persist throughout the
   * signal handling action.
   *
   * Keeping this data in the TCB resolves a security problem in protected
   * and kernel mode:  The regs[] array is visible on the user stack and
   * could be modified by a hostile program.
   */

  regs[XCPT_PC]        = rtcb->xcp.saved_pc;
  regs[XCPT_IRQCTL]    = rtcb->xcp.saved_irqctl;
  rtcb->xcp.sigdeliver = NULL;  /* Allows next handler to be scheduled */

  /* Then restore the correct state for this thread of execution. */

  board_autoled_off(LED_SIGNAL);
  z8_restorecontext(regs);
#endif
}

#endif /* CONFIG_DISABLE_SIGNALS */
