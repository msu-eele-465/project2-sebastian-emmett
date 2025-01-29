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

            ; Call tx_1 and tx_0 in order to send the hex byte AD as well as tx_ack to the AD :D
            ; call    #i2c_tx_1
            ; call    #i2c_tx_0
            ; call    #i2c_tx_1
            ; call    #i2c_tx_0
            ; call    #i2c_tx_1
            ; call    #i2c_tx_1
            ; call    #i2c_tx_0
            ; call    #i2c_tx_1
            ; call    #i2c_tx_ack

            ; Call send_address and call tx_1 (temp until we get i2c_send_write_bit), then call rx_ack. Should be seen as D1 (11010001) with a nack
            call    #i2c_send_address
            call    #i2c_tx_1
            call    #i2c_rx_ack

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
; i2c_tx_0 Subroutine: Transmit a '0' bit on our I2C bus
;------------------------------------------------------------------------------
i2c_tx_0:
        ; Transmit a '0' bit by setting SDA low and toggling SCL
        ; Flow:
        ;   1. Set SDA low.
        ;   2. Call i2c_sda_delay.
        ;   3. Set SCL high.
        ;   4. Call i2c_scl_delay.
        ;   5. Set SCL low.
        ;   6. Call i2c_scl_delay.

        ; Set SDA low
        bic.b   #BIT1, &P6OUT    ; Set SDA low
        call    #i2c_sda_delay    ; Delay after setting SDA low

        ; Set SCL high
        bis.b   #BIT0, &P6OUT    ; Set SCL high
        call    #i2c_scl_delay    ; Delay after setting SCL high

        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_tx_1 Subroutine: Transmit a '1' bit on our I2C bus
;------------------------------------------------------------------------------
i2c_tx_1:
        ; Transmit a '1' bit by setting SDA high and toggling SCL
        ; Flow:
        ;   1. Set SDA high.
        ;   2. Call i2c_sda_delay.
        ;   3. Set SCL high.
        ;   4. Call i2c_scl_delay.
        ;   5. Set SCL low.
        ;   6. Call i2c_scl_delay.

        ; Set SDA high
        bis.b   #BIT1, &P6OUT    ; Set SDA high
        call    #i2c_sda_delay    ; Delay after setting SDA high

        ; Set SCL high
        bis.b   #BIT0, &P6OUT    ; Set SCL high
        call    #i2c_scl_delay    ; Delay after setting SCL high

        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_tx_ack Subroutine: Transmit a '0' bit on our I2C bus for the acknowledge
;------------------------------------------------------------------------------
i2c_tx_ack:
        ; Transmit a '0' by just calling our i2c_tx_0 lmao
        ; Flow:
        ;   1. Call i2c_tx_0

        ; Set SDA high
        call    #i2c_tx_0        ; Call i2c_tx_0 to send a 0

        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_rx_ack Subroutine: Receive ACK bit from I2C slave
;------------------------------------------------------------------------------
i2c_rx_ack:
        ; Receive the acknowledgment bit from the I2C slave device after sending a byte
        ; Flow:
        ;   1.  Configure SDA as input.
        ;   2.  Release SDA (set high).
        ;   3.  Call i2c_sda_delay.
        ;   4.  Set SCL high to clock the ACK bit.
        ;   5.  Call i2c_scl_delay.
        ;   6.  Read SDA.
        ;   7.  Store ACK status in 'acknowledge'. - Admittedly more complicated than we first thought lol : need to update flowchart
        ;   8.  Set SCL low.
        ;   9.  Call i2c_scl_delay.
        ;   10. Configure SDA back to output.

        ; 1. Configure SDA as input
        bic.b   #BIT1, &P6DIR    ; Set P6.1 as input

        ; 2. Release SDA (set high)
        bis.b   #BIT1, &P6OUT    ; Release SDA (set high)

        ; 3. Call i2c_sda_delay
        call    #i2c_sda_delay

        ; 4. Set SCL high to clock the ACK bit
        bis.b   #BIT0, &P6OUT    ; Set P6.0 (SCL) high

        ; 5. Call i2c_scl_delay
        call    #i2c_scl_delay

        ; Okay now here is where I don't know if this approach will work. Does the slave
        ; hold SDA high until the clock cycles? I'd assume so and if so this will work.
        ; If they only hold it for some amount of time this is going to need reworked.
        ; Not exactly easy to test right now but will later.

        ; 6. Read SDA (bitwise AND - easiest approach imo)
        bit.b   #BIT1, &P6IN      ; Test P6.1 (SDA)

        ; 7. Store ACK status in 'acknowledge'
        jz      ACK_RECEIVED      ; If SDA is low, ACK received
        mov.b   #1, &acknowledge  ; Set 'acknowledge' to 1 (NACK)
        jmp     ACK_DONE

ACK_RECEIVED:
        mov.b   #0, &acknowledge  ; Set 'acknowledge' to 0 (ACK)

ACK_DONE:
        ; 8. Set SCL low
        bic.b   #BIT0, &P6OUT     ; Clear P6.0 (SCL) low

        ; 9. Call i2c_scl_delay
        call    #i2c_scl_delay

        ; 10. Configure SDA back to output
        bis.b   #BIT1, &P6DIR     ; Set P6.1 as output

        ret                       ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_send_address Subroutine: Send 7-bit I2C Device Address
;------------------------------------------------------------------------------
i2c_send_address:
        ; Send the 7-bit I2C device address one bit at a time
        ; Flow:
        ;   1.  Initialize loop counter to 7.
        ;   2.  Loop 7 times:
        ;       a.  Check MSB of device_address.
        ;       b.  If 0, call i2c_tx_0.
        ;           If 1, call i2c_tx_1.
        ;       c.  Rotate device_address left with carry.
        ;   3.  Repeat until all 7 address bits are sent.

        mov.b   &device_address, R14    ; Load device_address into R14
        mov     #7, R15                  ; Initialize loop counter to 7

send_address_loop:
        ; a. Check MSB (bit 6) of device_address
        ; Since device_address is 7-bit, MSB is bit 6
        bit.b   #BIT6, R14              ; Test bit 6 of R14
        jnz     send_bit_1               ; If bit 6 is 1, jump to send_bit_1

        ; b. If bit 6 is 0, transmit a 0 :D
        call    #i2c_tx_0

        jmp     rotate_address           ; Jump to rotate_address

send_bit_1:
        ; b. If bit 6 is 1, transmit a 1 :D
        call    #i2c_tx_1

rotate_address:
        ; c. Rotate device_address left with carry
        rlc.b   R14                      ; Rotate left through carry
        ; mov.b   R14, &device_address   ; Store rotated value back to device_address
        ; Leaving the above out since I'm not sure we want to actually update it or not.

        dec     R15                      ; Decrement loop counter
        jnz     send_address_loop        ; If not zero, continue loop

        ret                              ; Return from subroutine

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

;------------------------------------------------------------------------------
; Data Section: Define Variables :D
;------------------------------------------------------------------------------
        .sect   .data
        .global acknowledge
acknowledge:                        ; Yes I know it's a whole byte but I'm pretty sure a bit would be allocated the same space iirc
        .byte   0                   ; Initialize 'acknowledge' to 0 since nothing has been recieved yet

device_address:
        .byte   0x68                ; Default address for our DS3231