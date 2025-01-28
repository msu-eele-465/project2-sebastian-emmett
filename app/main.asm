;******************************************************************************
;   MSP430FR2355 Bitbanging I2C:
;   By Emmett & Sebastian
;******************************************************************************
        
        .cdecls  C,LIST,"msp430.h" ; Include device header file
;------------------------------------------------------------------------------
        .def    RESET              ; Export program entry point to linker
        .global __STACK_END
        .sect   .stack
;------------------------------------------------------------------------------

        .text
        .retain
        .retainrefs

;------------------------------------------------------------------------------
; Reset vector: set up stack pointer
;------------------------------------------------------------------------------
RESET:
        mov.w   #__STACK_END, SP    ; Initialize stack pointer

;------------------------------------------------------------------------------
; Initialization
;------------------------------------------------------------------------------
init:
        ; Stop the watchdog timer
        mov.w   #WDTPW+WDTHOLD, &WDTCTL

        ; Disable the GPIO high-impedance mode (low-power mode)
        bic.w   #LOCKLPM5, &PM5CTL0

        ; Configure P1.0 as output
        bis.b   #BIT0, &P1DIR        ; Set P1.0 (bit 0) as output

        ; Configure Timer_B0
        mov.w   #TBSSEL__ACLK+MC__UP+TBCLR, &TB0CTL  ; ACLK, Up mode, Clear
        mov.w   #32800, &TB0CCR0                     ; Set period for 1 second at ~32.800kHz (32.767 was slightly too slow)
        mov.w   #CCIE, &TB0CCTL0                     ; Enable CCR0 interrupt

        ; Enable global interrupts
        bis.w   #GIE, SR                             ; I know this gives a warning but it doesn't cause a issue- yet. Will fix if necessary later.

;------------------------------------------------------------------------------
; Main loop
;------------------------------------------------------------------------------
main:
        nop
        jmp     main
        nop

;------------------------------------------------------------------------------
; HEARTBEAT : Timer0_B0 : Interrupt Service Routine
;------------------------------------------------------------------------------
TIMER0_B0_ISR:
        ; Toggle P1.0
        xor.b   #BIT0, &P1OUT

        ; Return from interrupt
        reti

;------------------------------------------------------------------------------
; i2c_sda_delay Subroutine
;------------------------------------------------------------------------------
i2c_sda_delay:
        ; Delay for approximately 100 microseconds. Each ittr takes ~4 cycles (1 NOP + 1 DEC + 2 JNZ)
        ; I know this isn't going to be perfectly accurate but it should work for now. Total iterations: 25 → 25 * 4 = 100 cycles = 100µs

        mov     #25, R12          ; Initialize loop counter

delay_loop:
        nop                      ; 1 cycle (hehe I like this trick)
        dec     R12              ; 1 cycle
        jnz     delay_loop       ; 2 cycles if not zero
        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; Interrupt Vectors
;------------------------------------------------------------------------------
        .sect   RESET_VECTOR
        .short  RESET

        .sect   TIMER0_B0_VECTOR
        .short  TIMER0_B0_ISR
