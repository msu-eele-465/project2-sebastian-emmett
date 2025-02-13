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
init_rtc:
			mov.b	#068h, device_address			; device address of rtc is #068h

			; set time to 11:02:30 (for an arbitrary reason)
			mov.b	#000h, target_register			; set starting register to 0 (seconds)
			call	#i2c_write_start				; begin writing

			mov.b	#030h, new_value				; 30 seconds
			call	#i2c_write_transmit

			mov.b	#002h, new_value				; 2 minutes
			call	#i2c_write_transmit

			mov.b	#011h, new_value				; 11 hours
			call	#i2c_write_transmit_and_stop

read_time:
			mov.b	#00h, target_register			; set starting register to 0 (seconds)
			call	#i2c_read_start					; begin i2c read operation

			; read seconds
			call	#i2c_read_receive
			mov.b	output_value, R5

			; read minutes
			call	#i2c_read_receive
			mov.b	output_value, R6

			; read hours
			call	#i2c_read_receive_and_stop
			mov.b	output_value, R7

			; test_buffer will likely increment throughout the program due to the seconds incrementing
			; write arbitrary [6]
			mov.b	#000h, target_register
			mov.w	#test_buffer, bytes_buffer
			mov.b	#06d, bytes_buffer_size
			call	#i2c_write_arbitrary

			; write arbitrary [3]
			mov.b	#03d, bytes_buffer_size
			call	#i2c_write_arbitrary

			; write arbitrary [1]
			mov.b	#01d, bytes_buffer_size
			call	#i2c_write_arbitrary

			; read arbitrary [6]
			mov.b	#06d, bytes_buffer_size
			call	#i2c_read_arbitrary

			; read arbitrary [3]
			mov.b	#03d, bytes_buffer_size
			call	#i2c_read_arbitrary

			; read arbitrary [1]
			mov.b	#01d, bytes_buffer_size
			call	#i2c_read_arbitrary
      
      		; read temp
	      	call    #read_temperature

main_stop:

            ; Delay again (for visibility)
            call    #main_delay

            ; Repeat
            jmp     read_time
            nop

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


        ; Set SCL high
        bis.b   #BIT0, &P6OUT    ; Set SCL high (P6.0)
        call    #i2c_scl_delay    ; Delay after setting SCL high

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

        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

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
        
        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

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

        ; Set SCL low
        bic.b   #BIT0, &P6OUT    ; Set SCL low
        call    #i2c_scl_delay    ; Delay after setting SCL low

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

        call    #i2c_tx_0        ; Call i2c_tx_0 to send a 0

        ret                      ; Return from subroutine

;------------------------------------------------------------------------------
; i2c_tx_nack Subroutine: Transmit a '1' bit on our I2C bus for the NOT acknowledge
;------------------------------------------------------------------------------
i2c_tx_nack:
        ; Transmit a '1' by just calling our i2c_tx_1 lmao
        ; Flow:
        ;   1. Call i2c_tx_1

        call    #i2c_tx_1        ; Call i2c_tx_1 to send a 1

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
        bic.b	#BIT1, &P6DIR	; set as input
		bis.b	#BIT1, &P6REN	; enable resistor
		bis.b	#BIT1, &P6OUT	; pull-up resistor

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
        bis.b	#BIT1, &P6DIR	; set as output
        bic.b	#BIT1, &P6REN	; disable resistor (not sure if this needs to be disabled)

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
; i2c_tx_byte Subroutine: Send 'transmit_value' over i2c line
;------------------------------------------------------------------------------
i2c_tx_byte:

        mov.b   &transmit_value, R14    	; move transmit_value to R14 so it can be shifted out

        mov     #8d, R15                  	; set loop for 8 counts to shift all eight bits out

i2c_tx_byte_loop:
        bit.b   #BIT7, R14					; check the MSB of R14
        jnz     i2c_tx_byte_send_1        	; if MSB is 1, send a 1
        									; otherwise send a 0

		call	#i2c_tx_0	; transmit 0

        jmp     i2c_tx_byte_rotate			; rotate R14 to prepare for next loop

i2c_tx_byte_send_1:
        call    #i2c_tx_1	; transmit 1

i2c_tx_byte_rotate:
        rlc.b   R14                      	; rotate left w carry, so next loop can grab next bit

        dec     R15                      	; decrement loop counter
        jnz     i2c_tx_byte_loop        	; loop through all 8 bits

        call	#i2c_rx_ack					; receive acknowledge from slave
        bit.b	#BIT0, &acknowledge

        jnz		i2c_tx_byte					; acknowledge not received, repeat subroutine
        									; otherwise, ackowledge received, terminate subroutine

        ret

;------------------------------------------------------------------------------
; i2c_rx_byte Subroutine: Receive 1 byte over the i2c line and store it in output_value
;------------------------------------------------------------------------------
i2c_rx_byte:

		; set P6.1, SDA, as an input
		bic.b	#BIT1, &P6DIR	; set as input
		bis.b	#BIT1, &P6REN	; enable resistor
		bis.b	#BIT1, &P6OUT	; pull-up resistor

        mov.b  	#0d, R14    				; designate R14 as buffer to hold output

        mov     #8d, R15                  	; set loop for 8 counts to shift all eight bits in

i2c_rx_byte_loop:
		rla.b	R14							; shift all current bits left

        call    #i2c_sda_delay				; delay for slave on SDA

        ; set SCL high
        bis.b   #BIT0, &P6OUT
        call    #i2c_scl_delay

        bit.b   #BIT1, &P6IN				; check the value of SDA
        jz     	i2c_rx_byte_noinc       	; if SDA is 1, increment R14
        									; otherwise don't

        inc.b	R14

i2c_rx_byte_noinc:
        ; set SCL low
        bic.b   #BIT0, &P6OUT
        call    #i2c_scl_delay

        dec     R15                      	; decrement loop counter
        jnz     i2c_rx_byte_loop        	; loop through all 8 bits

        mov.b	R14, &output_value			; update the read value

        ; set P6.1, SDA, as an output
        bis.b	#BIT1, &P6DIR	; set as output
        bic.b	#BIT1, &P6REN	; disable resistor (not sure if this needs to be disabled)

        call    #i2c_scl_delay

        ; NOTE: Cannot tx ack unless we're NOT done sending data. tx_ack logic should be outside this subroutine.
        ; call	#i2c_tx_ack					; transmit acknowledge DONT DO THIS HERE

        ret

;------------------------------------------------------------------------------
; i2c_send_read_bit Subroutine: Send the 8th bit in the i2c header as a 1
;------------------------------------------------------------------------------
i2c_send_read_bit:

        call	#i2c_tx_1		; transmit a 1 over the i2c line, which corresponds to a read request

        call	#i2c_rx_ack		; wait for an acknowledge from the slave device

        ret

;------------------------------------------------------------------------------
; i2c_send_write_bit Subroutine: Send the 8th bit in the i2c header as a 0
;------------------------------------------------------------------------------
i2c_send_write_bit:

        call	#i2c_tx_0		; transmit a 0 over the i2c line, which corresponds to a write request

        call	#i2c_rx_ack		; wait for an acknowledge from the slave device

        ret

;------------------------------------------------------------------------------
; i2c_read_start Subroutine: Start an i2c read operation on the desired register
;
; device_address and target_register should be set up before calling this
;
; output_value will store the read value
;------------------------------------------------------------------------------
i2c_read_start:

    	; select the register to read from
        call    #i2c_write_start			; use i2c_write_start to select the register
		call	#i2c_stop					; end write operation

i2c_read_header_read:
		; generate I2C start condition
        call    #i2c_start

		; send i2c header information for read operation
        call    #i2c_send_address
        call    #i2c_send_read_bit

        bit.b	#BIT0, &acknowledge			; verify acknowledge value
        jz		i2c_read_continue			; if acknowledged, continue
        									; otherwise, resend the header after a stop condition

        ; generate I2C stop condition, before resending the same header
        call	#i2c_stop
        jmp		i2c_read_header_read

i2c_read_continue:

		ret


;------------------------------------------------------------------------------
; i2c_read_receive Subroutine: Read one byte from the current register and prepare for more
;
; i2c_read_start should be called before this subroutine
;
; output_value will store the read value
;------------------------------------------------------------------------------
i2c_read_receive
        call    #i2c_rx_byte				; read value from register
        call    #i2c_tx_ack					; transmit ACK, since we will want more data

        ret

;------------------------------------------------------------------------------
; i2c_read_receive_and_stop Subroutine: Read one byte from the current register and end read operation
;
; i2c_read_start should be called before this subroutine
;
; output_value will store the read value
;------------------------------------------------------------------------------
i2c_read_receive_and_stop
        call    #i2c_rx_byte				; read value from register
        call    #i2c_tx_nack				; transmit NACK, since we want no more data

        call    #i2c_stop					; generate I2C stop condition to end read operation

        ret

;------------------------------------------------------------------------------
; i2c_write_start Subroutine: Select a register to start an i2c write operation on
;
; device_address and target_register should be set up before calling this
;------------------------------------------------------------------------------
i2c_write_start:
i2c_write_header_write:

    	; generate I2C start condition
        call    #i2c_start

		; send i2c header information for write operation
        call    #i2c_send_address
        call    #i2c_send_write_bit

        bit.b	#BIT0, &acknowledge			; verify acknowledge value
        jz		i2c_write_continue			; if acknowledged, continue
        									; otherwise, resend the header after a stop condition

        ; generate I2C stop condition, before resending the same header
        call	#i2c_stop
        jmp		i2c_write_header_write

i2c_write_continue:
		; set up register
		mov.b	target_register, transmit_value		; prepare to send desired register to write to

		call	#i2c_tx_byte				; send register to write to

		ret

;------------------------------------------------------------------------------
; i2c_write_transmit Subroutine: Send a byte to the current register
;
; i2c_write_start should be called before this subroutine
;
; new_value should be setup and contain the byte to write to the register
;------------------------------------------------------------------------------

i2c_write_transmit
		; write to register
		mov.b	new_value, transmit_value	; prepare to send desired value

		call	#i2c_tx_byte				; send value to register

        ret

;------------------------------------------------------------------------------
; i2c_write_transmit_and_stop Subroutine: Send a byte to the current register and stop writing
;
; new_value should be setup and contain the byte to write to the register
;------------------------------------------------------------------------------

i2c_write_transmit_and_stop
		; write to register
		mov.b	new_value, transmit_value	; prepare to send desired value

		call	#i2c_tx_byte				; send value to register
		call	#i2c_stop					; end write operation

        ret

;------------------------------------------------------------------------------
; read_temperature reads the temparature value from the RTC and stores it (LSB and MSB stored seperately)
;
; calls decode_temperature
;------------------------------------------------------------------------------

read_temperature:
        ; We want to read from register 0x11 (MSB) and 0x12 (LSB)
        mov.b   #0x11, target_register
        call    #i2c_read_start

        ; Read the MSB (integer portion + sign bit)
        call    #i2c_read_receive
        mov.b   output_value, &temp_msb

        ; Read the LSB (fractional bits in upper nibble)
        call    #i2c_read_receive_and_stop
        mov.b   output_value, &temp_lsb

        ; We can do the temp conversion here if we want to store it some other way. #TODO
        ; okay so I did it real quick but idk if it works lmao
        call    #decode_temperature

        ret

;------------------------------------------------------------------------------
; decode_temperature Subroutine
;------------------------------------------------------------------------------
;
; Input:
;   - temp_msb (byte in memory at &temp_msb)
;   - temp_lsb (byte in memory at &temp_lsb)
;     * bits [7..6] are fractional portion, i.e. 0.25°C increments
;     * bit 7 of temp_msb is the sign bit (two's comp)
;
; Output:
;   - R8 (16-bit). The lower 10 bits contain the sign-extended result.
;     Each unit = 0.25°C.
;     Positive or negative properly sign-extended.
;
; Example:
;   MSB=0x19 (0001_1001), LSB=0x40 (0100_0000 => fractional bits=01)
;   => +25.25°C => R8 = 25 * 4 + 1 = 101 decimal = 0x0065
;   If MSB had bit 7 set => negative temperature => sign-extended.
;
;------------------------------------------------------------------------------
decode_temperature:
    ; 1. Load raw bytes into registers R14 and R15
    mov.b   &temp_msb, R14
    mov.b   &temp_lsb, R15

    ; 2. Shift the MSB left by 2 bits.
    ;    That moves the integer portion to bits [8..2] and the sign bit to bit 9.
    rla.w   R14             ; Shift left by 1
    rla.w   R14             ; Shift left by another 1  => total of 2 bits

    ; 3. Isolate the 2 fractional bits from temp_lsb by shifting them right 6 times.
    ;    That puts them into bits [1..0] of R15. We only need 2 bits, but DS3231 docs say
    ;    those are bits [7..6], so we shift down 6 :D
    mov     #6, R13
dt_shift_lsb:
    rra.b   R15             ; shift right
    dec     R13
    jnz     dt_shift_lsb

    ; 4. Combine those 2 fractional bits into the low 2 bits of R14.
    or.b    R15, R14

    ; 5. Sign-extend if negative
    ;    Check bit7 of temp_msb (the original sign bit).
    ;    If set => temperature is negative => we set the upper 6 bits of R14 to 1.
    bit.b   #0x80, &temp_msb
    jz      dt_done         ; if 0 => not negative, skip sign extension

    ; Negative => set bits [15..10] = 1
    ; because bit9 is the sign bit after shifting,
    ; so we set all bits above bit9 to 1 => 0xFC00
    bis.w   #0xFC00, R14

dt_done:
    ; 6. Store final 16-bit result in R12
    ;    (Each increment is 0.25°C)
    mov.w   R14, R12

    ; Return
    ret

;------------------------------------------------------------------------------
; i2c_read_arbitrary Subroutine: Receive bytes_buffer_size bytes from i2c and store in
;	the location pointed to by bytes_buffer
;
; bytes_buffer, bytes_buffer_size, device_address and target_register should be set up before calling this
;------------------------------------------------------------------------------
i2c_read_arbitrary:
		call	#i2c_read_start					; begin i2c read operation

		mov.w	bytes_buffer, R11				; store location of buffer
		mov.b	bytes_buffer_size, R10			; store amount of bytes to read in

		; preemptively decrement by one so the last byte is not received by the loop
		; if R10 == 1 beforehand then skip the loop altogether
		dec.b	R10
		jz		i2c_read_arbitrary_last

i2c_read_arbitrary_loop:

		; read next register and store in buffer
		call	#i2c_read_receive
		mov.b	output_value, 0(R11)			; copy into first byte pointed to by R11
		inc.b	R11								; increment R11 pointer by one byte

		; test condition
		dec.b	R10
		jnz		i2c_read_arbitrary_loop			; if the second to last byte is not received repeat loop
												; otherwise, receive the last byte

i2c_read_arbitrary_last:
		; read next register, store in buffer, and terminate read operation
		call	#i2c_read_receive_and_stop
		mov.b	output_value, 0(R11)			; copy into first byte pointed to by R12

		ret

;------------------------------------------------------------------------------
; i2c_write_arbitrary Subroutine: Transmit bytes_buffer_size bytes through i2c and use
;	the location pointed to by bytes_buffer to acquire transmit data
;
; bytes_buffer, bytes_buffer_size, device_address and target_register should be set up before calling this
;------------------------------------------------------------------------------
i2c_write_arbitrary:
		call	#i2c_write_start				; begin i2c write operation

		mov.w	bytes_buffer, R11				; store location of buffer
		mov.b	bytes_buffer_size, R10			; store amount of bytes to read in

		; preemptively decrement by one so the last byte is not sent by the loop
		; if R10 == 1 beforehand then skip the loop altogether
		dec.b	R10
		jz		i2c_write_arbitrary_last

i2c_write_arbitrary_loop:

		; transmit next byte
		mov.b	@R11+, new_value				; prepare new value and increment pointer value
		call	#i2c_write_transmit

		; test condition
		dec.b	R10
		jnz		i2c_write_arbitrary_loop		; if the second to last byte is not sent repeat loop
												; otherwise, send the last byte

i2c_write_arbitrary_last:
		; transmit last value and terminate write operation
		mov.b	@R11, new_value
		call	#i2c_write_transmit_and_stop

		ret

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
; Data Section: Define Variables :D
;------------------------------------------------------------------------------

        .data
        .retain

acknowledge:                        ; Yes I know it's a whole byte but I'm pretty sure a bit would be allocated the same space iirc
        .byte   0                   ; Initialize 'acknowledge' to 0 since nothing has been recieved yet

device_address:
        .byte   0x68                ; Default address for our DS3231

target_register:
		.byte	0					; which register should an i2c read or write operation target

new_value:
		.byte	0					; what value should be written during an i2c write operation

transmit_value:
		.byte	0					; what value should tx_byte transmit

output_value:
		.byte	0					; output of i2c read operation

temp_msb:
        .byte   0					; most significant byte of temperature, each bit is 1 deg C

temp_lsb:
        .byte   0					; least significant byte of temperature, bit 7 and 6 are beyond the decimal point
        							; and represent a portion of a deg C

bytes_buffer:
		.short	0					; a buffer to either write out or read in through i2c
									; the value stored here should be a pointer to the buffer being used (reserved memory)

bytes_buffer_size:
		.byte	0					; an amount of bytes to read/write out through i2c
									; to prevent overflow this should not exceed the bytes_buffers' size

test_buffer:
		.byte	1, 2, 3, 4, 5, 6	; a buffer for use by i2c arbitrary read/write

;------------------------------------------------------------------------------
; HEARTBEAT : Timer0_B0 : Interrupt Service Routine
;------------------------------------------------------------------------------
TIMER0_B0_ISR:
        ; Toggle P1.0
        xor.b   #BIT0, &P1OUT

        ; Return from interrupt
        reti

;------------------------------------------------------------------------------
; Interrupt Vectors
;------------------------------------------------------------------------------
        .sect   RESET_VECTOR
        .short  RESET

        .sect   TIMER0_B0_VECTOR
        .short  TIMER0_B0_ISR
