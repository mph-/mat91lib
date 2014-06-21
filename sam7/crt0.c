/** @file   crt0.c
    @author M. P. Hayes, UCECE
    @date   10 June 2007
    @brief  C run time initialisation for the Atmel AT91 series 
            of ARM7 TDMI microprocessors. 
*/

#include "config.h"
#include "cpu.h"
#include "mcu.h"

/* This requires cpu.h, irq.h, and mcu.h of mat91lib. 

   The user application is run in Supervisor Mode.  This allows full
   privileges.  The top 72 bytes of SRAM are used as the IRQ Mode
   stack (this does not have to be large since the interrupt handler
   switches to Supervisor Mode).  The Supervisor Mode stack sits below
   this.  FIQ Mode has no allocated stack.

   Usually this is written in assembler; I have been bloody minded and
   wrote it in C using a fair bit of inline assembler when necessary.
*/


#ifndef IRQ_STACK_SIZE
/* Store 3 registers R0, LR, and SPSR during IRQ.  */
#define IRQ_STACK_SIZE  (3 * 8 * 4)
#endif


#ifndef ABT_STACK_SIZE
/* Store 2 registers  LR and SPSR during ABORT.  */
#define ABT_STACK_SIZE  (2 * 4 * 2)
#endif


#define AIC_FVR_OFFSET 0x104
#define AIC_IVR_OFFSET 0x100
//#define AIC_FVR_OFFSET ((char *)&AT91C_BASE_AIC->AIC_FVR - (char *)AT91C_BASE_AIC)
//#define AIC_IVR_OFFSET ((char *)&AT91C_BASE_AIC->AIC_IVR - (char *)AT91C_BASE_AIC)


/** Symbols defined by linker script.  These are all VMAs except those
    with a _load__ suffix which are LMAs.  */
extern char __stack_start__;    /** Top of stack.  */
extern char __data_load__;      /** Start of initial values for .data (in flash).  */
extern char __data_start__;     /** Start of data (in SRAM).  */
extern char __data_end__;       /** End of data (in SRAM).  */
extern char __bss_start__;      /** Start of uninitialised variables.  */
extern char __bss_end__;        /** End of uninitialised variables.  */


extern int main (int, char *);


void exit (int status __attribute ((unused)))
{
    while (1)
        continue;
}


extern void start (void) __attribute__ ((section (".vectors")))
    __attribute__ ((naked));

/** Startup function.  */
void start (void)
{
    /* Vector table.  Note the branch instructions are relative and
       thus the PC will stay in the first bank of memory and thus
       confuse the debugger.  We could do an absolute jump but this will
       need an intermediate handler for each exception.  */

    /* Reset  0x00.  */
    __asm__ ("\tb _reset_handler");
    /* Hang if decoded undefined instruction  0x04.  */
    __asm__ ("\tb ."); 
    /* Hang if software interrupt 0x08.  */
    __asm__ ("\tb .");
    /* Fetched instruction from invalid address (Prefetch Abort) 0x0c.
       LR - 8 points to the instruction that caused the abort.  SP is
       for abort mode.  */
    __asm__ ("\tb _prefetch_abort_handler"); 
    /* Invalid address (or alignment) (Data Abort) 0x10.  LR - 8
       points to the instruction that caused the abort.  SP is for
       abort mode.  */ 
    __asm__ ("\tb _data_abort_handler"); 
    /* Reserved  0x14.  */
    __asm__ ("\tb .");
    /* IRQ 0x18.  */
    __asm__ ("\tb _irq_handler");

    /* Fall through to FIQ handler.  */
}


extern void _fiq_handler (void)
    __attribute__ ((section (".vectors")))
    __attribute__ ((naked));

void _fiq_handler (void)
{

    /* In FIQ Mode we have banked registers R8--R14 and thus can hold
       values between fast interrupts.  R13 is reserved for the stack
       pointer and R14 is reserved for the link register.

       We use R8 to point to the base of the AIC.  */

    /* Save r0 in FIQ_Register.  */
    __asm__ ("\tmov r9, r0");

    
    __asm__ ("\tldr r0, [r8, %0]" : : "i" (AIC_FVR_OFFSET));
         
    
    /* Switch to Supervisor Mode to allow stack access for C code
       because the FIQ is not yet acknowledged.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_F_BIT | CPU_MODE_SVC);

    /* Save scratch/used registers and LR on Supervisor Mode stack.  */
    __asm__ ("\tstmfd sp!, {r1-r3, r12, lr}");

    /* Branch to the routine pointed by the AIC_FVR.  */
    __asm__ ("\tmov r14, pc");
    __asm__ ("\tbx r0");

    /* Restore scratch/used registers and LR from Supervisor Mode stack.  */
    __asm__ ("\tldmia sp!, {r1-r3, r12, lr}");

    /* Leave interrupts disabled and switch back to FIQ Mode.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_F_BIT | CPU_MODE_FIQ);

    /* Restore the R0 CPU_MODE_SVC register.  */
    __asm__ ("\tmov r0, r9");

    /* Restore the Program Counter using the LR_FIQ directly in the PC.  */
    __asm__ ("\tsubs pc, lr, #4");
}


/** IRQ handler.  This is called in IRQ mode when nIRQ is asserted and
    the I bit in the CPSR is clear.  On entry the I bit is
    automatically set to prevent further interrupts.  This function
    must be compiled in ARM mode because of some of the inline ARM
    assembler instructions that are not supported in Thumb mode.  Note
    that a breakpoint cannot be put directly on this function; when an
    interrupt occurs the PC is set to 0x18.  A relative branch is then
    performed to this function so the PC will be something like 0x264.
    This address will then be mapped into flash (0x100264) or SRAM
    (0x200264) and the instructions found.  The debugger, however,
    will try putting the breakpoint at 0x100264 or 0x200264 but this
    will not work.  */
extern void _irq_handler (void)
    __attribute__ ((section (".vectors")))
    __attribute__ ((naked));

void _irq_handler (void)
{
    /* In IRQ mode we have banked registers R13--R14.  R13 is reserved
       for the stack pointer (SP) and R14 is reserved for the link
       register (LR).  */

    /* Adjust and save LR_irq in IRQ stack.  */
    __asm__ ("\tsub lr, lr, #4");
    __asm__ ("\tstmfd sp!, {lr}");

    /* Save SPSR for nested interrupt.  */
    __asm__ ("\tmrs r14, SPSR");
    __asm__ ("\tstmfd sp!, {r14}");

    /* Save r0 in IRQ stack.  */
    __asm__ ("\tstmfd sp!, {r0}");

    /* Read AIC_IVR.  This sets the current interrupt to be the
      pending and enabled interrupt with the highest priority.  The
      current level is the priority level of the current interrupt.
      This also de-asserts the nIRQ line on the processor and clears
      the interrupt.  The current level and the current interrupt
      number is pushed on to the stack.  Finally, the value written in
      the AIC_SVR corresponding to the current interrupt is
      returned. 

      When using a debugger spurious interrupts can be generated.
      This can be avoided using protect mode.  In this mode interrupt
      stacking is only performed when the AIC_AVR is written to.  */
    {
        /* Use r14 (LR) since it is saved.  */
        register uint32_t r14 __asm__ ("r14") = (uint32_t) AT91C_BASE_AIC;

        __asm__ ("\tldr r0, [%1, %0]" : : "i" (AIC_IVR_OFFSET), "r" (r14));

        /* This line can be removed if protect mode not required.  */
        __asm__ ("\tstr %1, [%1, %0]" : : "i" (AIC_IVR_OFFSET), "r" (r14));
    }

    /* Enable interrupts and switch back to Supervisor Mode.  This
       allows nested interrupts.  */
    cpu_cpsr_c_set_const (CPU_MODE_SVC);

    /* Save scratch/used registers and LR on Supervisor Mode stack.  */
    __asm__ ("\tstmfd sp!, {r1-r3, r12, r14}");

    /* Branch to the routine pointed by the AIC_IVR.  This routine
       runs in Supervisor Mode.  */
    __asm__ ("\tmov r14, pc");
    __asm__ ("\tbx r0");

    /* Restore scratch/used registers and LR from Supervisor Mode stack.  */
    __asm__ ("\tldmia sp!, {r1-r3, r12, r14}");

    /* Disable interrupt and switch back in IRQ Mode.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_MODE_IRQ);

    /* Mark the End of Interrupt on the AIC.  */
    {
        register uint32_t * r14 __asm__ ("r14") 
            = (uint32_t *) &AT91C_BASE_AIC->AIC_EOICR;

        /* Write anything to AIC_EOICR to indicate interrupt handling
           complete.  */
        __asm__ ("\tstr %0, [%0, #0]" : : "r" (r14));
    }

    /* Restore SPSR_irq and r0 from IRQ Mode stack.  */
    __asm__ ("\tldmia sp!, {r0}");
    __asm__ ("\tldmia sp!, {r14}");
    __asm__ ("\tmsr SPSR_cxsf, r14");

    /* Restore adjusted LR_irq from IRQ Mode stack directly in the PC.
       This causes a return from interrupt.  */
    __asm__ ("\tldmia sp!, {pc}^");
}


/** Dummy ISR for unexpected interrupts.  */
void
_irq_unexpected_handler (void)
{
    while (1)
        continue;
}


static int spurious_interrupts = 0;


/** Dummy ISR for spurious interrupts.  */
void
_irq_spurious_handler (void)
{
    /* Spurious interrupts are unavoidable but harmless.  They can occur when
       an interrupt is in the pipeline and then interrupts are disabled.  */
    spurious_interrupts++;
}


extern void reset (void)
    __attribute__ ((alias ("_reset_handler")));


extern void _reset_handler (void)
    __attribute__ ((section (".vectors")))
    __attribute__ ((naked));


void _reset_handler (void)
{
    register char *p = &__stack_start__;
#if 1
    /* Set up temporary stack in top of internal RAM.   */
    register uint32_t sp __asm__ ("sp") = (uint32_t)p;
    register uint32_t fp __asm__ ("fp") = (uint32_t)p;

    /* Create dummy uses.  */
    __asm__ ("" : : "r" (sp));
    __asm__ ("" : : "r" (fp));
#else
    /* This does not work without optimisation since it requires the
       stack and frame pointer to be set up.  */       
    cpu_sp_set ((uint32_t) p);
#endif

    /* Start clocks, map SRAM, etc.  */
    mcu_init ();

    /* Select FIQ Mode and disable interrupts.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_F_BIT | CPU_MODE_FIQ);

    /* Initialise the FIQ register to point to the AIC.  */
    {
        register uint32_t r8 __asm__ ("r8") = (uint32_t) AT91C_BASE_AIC;

        /* Create a dummy use.  */
        __asm__ ("" : : "r" (r8));
    }

    /* Select IRQ Mode and disable interrupts.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_F_BIT | CPU_MODE_IRQ);

    /* Set up IRQ Mode stack.  */
    cpu_sp_set ((uint32_t) p);


    /* Select Abort Mode and disable interrupts.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_F_BIT | CPU_MODE_ABT);

    /* Set up Abort Mode stack.  */
    cpu_sp_set ((uint32_t) p - IRQ_STACK_SIZE);


    /* Select Supervisor Mode and enable interrupts.  */
    cpu_cpsr_c_set_const (CPU_MODE_SVC);    

    /* Set up Supervisor Mode stack.  */
    cpu_sp_set ((uint32_t) (p - IRQ_STACK_SIZE - ABT_STACK_SIZE));

    {
        char *src;
        char *dst;

        /* Initialise initialised global variables in .data and
           relocate .ramtext function for functions in the ROM model
           that need to execute out of RAM for speed.  */
        for (src = &__data_load__, dst = &__data_start__; dst < &__data_end__; )
            *dst++ = *src++;
        
        /* Zero uninitialised global variables in .bss.  */
        for (dst = &__bss_start__; dst < &__bss_end__; )
            *dst++ = 0;
    }

    /* Call main using absolute addressing through the interworking.  */
    __asm__ ("\tldr lr, =exit");
    __asm__ ("\tldr r0, =main");
    __asm__ ("\tbx  r0");
}


/* For a detailed description of data aborts see
   http://www.embedded.com/192202641

   When a data abort occurs, the CPU switches to Abort Mode and the PC + 8
   of the offending instruction is saved in LR_ABT.  The PC_ABT is
   then loaded with the address of the Abort Exception vector.
*/

extern void _data_abort_handler (void)
    __attribute__ ((section (".vectors")));

void _data_abort_handler (void)
{
    /* In abort mode we have banked registers R13--R14.  R13 is reserved
       for the stack pointer (SP) and R14 is reserved for the link
       register (LR).  */

    /* Save LR_ABT in r0 (we could save r0 on abort stack).  */
    __asm__ ("\tmov r0, lr");
    __asm__ ("\tsub r0, r0, #4");

    /* Switch back to Supervisor Mode.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_MODE_SVC);

    __asm__ ("\tmov lr, r0");

    /* GDB cannot reconcile the PC with the address of a known
       function since the PC is currently pointing into the first
       block of memory.  */

    __asm__ ("\tldr pc, =_data_abort_hang"); 
}


void _data_abort_hang (void)
{
    /* Hang.  Don't use a while (1) otherwise the debug info will be invalid.  */
    __asm__ ("\tb ."); 
}


/* When a prefetch abort occurs, the CPU switches to Abort Mode and
   the PC + 4 of the offending instruction is saved in LR_ABT.  The
   PC_ABT is then loaded with the address of the Abort Exception
   vector.  */

extern void _prefetch_abort_handler (void)
    __attribute__ ((section (".vectors")));

void _prefetch_abort_handler (void)
{
    /* In abort mode we have banked registers R13--R14.  R13 is reserved
       for the stack pointer (SP) and R14 is reserved for the link
       register (LR).  */

    /* Save LR_ABT in r0 (we could save r0 on abort stack).  */
    __asm__ ("\tmov r0, lr");

    /* Switch back to Supervisor Mode.  */
    cpu_cpsr_c_set_const (CPU_I_BIT | CPU_MODE_SVC);

    __asm__ ("\tmov lr, r0");

    /* GDB cannot reconcile the PC with the address of a known
       function since the PC is currently pointing into the first
       block of memory.  */

    __asm__ ("\tldr pc, =_prefetch_abort_hang"); 
}


void _prefetch_abort_hang (void)
{
    /* Hang.  Don't use a while (1) otherwise the debug info will be invalid.  */
    __asm__ ("\tb ."); 
}
