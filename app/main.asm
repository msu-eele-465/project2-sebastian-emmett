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

        ; Configure P1.0 as output
        bis.b   #BIT0, &P1DIR        ; Set P1.0 (bit 0) as output

        ; Configure P6.0 (SCL) and P6.1 (SDA) for PUSH-PULL output
        bis.b   #BIT0 + BIT1, &P6DIR  ; Set P6.0 and P6.1 as outputs
        bis.b   #BIT0 + BIT1, &P6OUT  ; Drive both pins HIGH initially

        ; Explicitly select digital I/O for P6.0 and P6.1
        ; Clear P6SEL0 and P6SEL1 bits to select GPIO function
        bic.b   #BIT0 + BIT1, &P6SEL0 ; Clear P6SEL0 for P6.0 and P6.1
        bic.b   #BIT0 + BIT1, &P6SEL1 ; Clear P6SEL1 for P6.0 and P6.1

        ; Disable the GPIO high-impedance mode after GPIO config (low-power mode)
        bic.w   #LOCKLPM5, &PM5CTL0

        ; Configure Timer_B0
        mov.w   #TBSSEL__ACLK+MC__UP+TBCLR, &TB0CTL  ; ACLK, Up mode, Clear
        mov.w   #32800, &TB0CCR0                     ; Set period for 1 second at ~32.800kHz (32.767 was slightly too slow)
        mov.w   #CCIE, &TB0CCTL0                     ; Enable CCR0 interrupt

        ; Enable global interrupts
        nop
        eint
        nop

        ; Jump to main
        jmp     main

;------------------------------------------------------------------------------
; Main loop
;------------------------------------------------------------------------------
main:
            ; Generate I2C Start Condition
            call    #i2c_start

            ; Delay between start and stop (for visibility)
            call    #main_delay

            ; Generate I2C Stop Condition
            call    #i2c_stop

            ; Delay again (for visibility)
            call    #main_delay

            ; Repeat
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

delay_loop_sda:
        nop                      ; 1 cycle (hehe I like this trick)
        dec     R12              ; 1 cycle
        jnz     delay_loop_sda   ; 2 cycles if not zero
        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_scl_delay Subroutine
;------------------------------------------------------------------------------
i2c_scl_delay:
        ; Delay for approximately 100 microseconds. Each ittr takes ~4 cycles (1 NOP + 1 DEC + 2 JNZ)
        ; I know this isn't going to be perfectly accurate but it should work for now. Total iterations: 25 → 25 * 4 = 100 cycles = 100µs

        mov     #25, R12          ; Initialize loop counter (reusing R12 from sda_delay)

delay_loop_scl:
        nop                      ; 1 cycle (hehe yep this trick again)
        dec     R12              ; 1 cycle
        jnz     delay_loop_scl   ; 2 cycles if not zero
        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_start Subroutine
;------------------------------------------------------------------------------
i2c_start:
        ; I2C Start Condition
        ; Flow as we defined in our flowcharts:
        ;   1. Set SDA low.
        ;   2. Call i2c_sda_delay.
        ;   3. Set SCL low.
        ;   4. Call i2c_scl_delay.

        ; At some point we might need to ensure SDA is alr high here but ignoring for now

        ; Set SDA low
        bic.b   #BIT1, &P6OUT    ; Set SDA low
        call    #i2c_sda_delay    ; Delay after setting SDA low

        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

        ret                      ; Return from subroutine
    
;------------------------------------------------------------------------------
; i2c_stop Subroutine
;------------------------------------------------------------------------------
i2c_stop:
        ; I2C Stop Condition
        ; Flow:
        ;   1. Set SDA low.
        ;   2. Set SCL high.
        ;   3. Call i2c_scl_delay.
        ;   4. Set SDA high.
        ;   5. Call i2c_sda_delay.

        ; We might need to verify SCL is low here as well - skipping for now.

        ; Set SDA low
        bic.b   #BIT1, &P6OUT    ; Set SDA low (P6.1)
        call    #i2c_sda_delay    ; Delay after setting SDA low

        ; Set SCL high
        bis.b   #BIT0, &P6OUT    ; Set SCL high (P6.0)
        call    #i2c_scl_delay    ; Delay after setting SCL high

        ; Set SDA high
        bis.b   #BIT1, &P6OUT    ; Set SDA high (P6.1)
        call    #i2c_sda_delay    ; Delay after setting SDA high

        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; main_delay Subroutine (TESTING)
;------------------------------------------------------------------------------
main_delay:
        ; Delay for approximately 1 millisecond - Each iter takes ~4 cycles (1 NOP + 1 DEC + 2 JNZ)
        ; Total iterations: 250 → 250 * 4 = 1000 cycles = 1ms

        mov     #250, R13          ; Initialize loop counter

delay_loop_main:
        nop                      ; 1 cycle
        dec     R13              ; 1 cycle
        jnz     delay_loop_main  ; 2 cycles if not zero
        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; Interrupt Vectors
;------------------------------------------------------------------------------
        .sect   RESET_VECTOR
        .short  RESET

        .sect   TIMER0_B0_VECTOR
        .short  TIMER0_B0_ISR
