; LEDs.asm
; Runs on MSP432
; Capt Steven Beyer
; September 9, 2019


; Code to activate LED on P6.7. This code accompanies the Lab05_LED_Switchesmain.c

        .thumb

        .text
; Addresses for Port 1 registers
; Find the address values on Valvano, page 155.
; The dots between addresses are for easy reading.
; Do not add dots in your code.
        .align 4
P1SEL0  .word 0x40004C0A  ; replace 0 with the address
P1SEL1  .word 0x40004C0C  ; replace 0 with the address
P1DIR   .word 0x40004C04  ; replace 0 with the address
P1OUT   .word 0x40004C02  ; replace 0 with the address

; Use this delay for Homework 8.
; You need to adjust this value in Lab8.
DELAY   .word 1200000

        ; global functions external function can access these functions.
        .global LED_Init
        .global LED_Off
        .global LED_On
        .global LED_Toggle
        .global LED_Oscillate

; function to initialize P1.0 (LED)
LED_Init:       .asmfunc
        LDR     R1, P1SEL0		   ; Load the address of P1SEL0
        LDRB    R0, [R1]
        BIC     R0, R0, #0x01      ; Clear P1SEL0.0
        STRB    R0, [R1]
        LDR     R1, P1SEL1         ; Load the address of P1SEL1
        LDRB    R0, [R1]
        BIC     R0, R0, #0x01      ; Clear P1SEL1.0
        STRB    R0, [R1]           ; store byte
        LDR     R1, P1DIR
        LDRB    R0, [R1]           ; load byte
        ORR     R0, R0, #0x01      ; bit set
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to turn off LED (P1.0)
LED_Off:        .asmfunc
        LDR     R1, P1OUT
        LDRB    R0, [R1]        ; 8-bit read
        BIC     R0, R0, #0x01   ; turn off
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to turn on P1.0
LED_On:         .asmfunc
        LDR     R1, P1OUT
        LDRB    R0, [R1]       ; 8-bit read
        ORR     R0, R0, #0x01   ; turn on
        STRB    R0, [R1]
        BX      LR
        .endasmfunc


; function to toggle P1.0
; Turn off LED if it is on.
; Turn on LED if it is off.
; To toggle a bit, you don't need to know whether it is currently on or off.
; Do not use LED_Off or LED_On
LED_Toggle: 	.asmfunc
        LDR     R0, P1OUT   ; Read P1 to R0
        LDRB    R1, [R0]    ; Copy bit to R1 for a bit of cheeky masking
        EOR     R1, R1, #1  ; XOR so it sets pin 1
        STRB    R1, [R0]    ; Write upadated value
        BX      LR          ; return from whence we came

        .endasmfunc


; function to continuously toggle P1.0 every half second
; use a loop as a timer
LED_Oscillate:  .asmfunc
; add your code here

Begin   BL      LED_Toggle  ; togles
        LDR     R0, DELAY   ; loads the delay constant in R0

Loop1   SUB     R0, R0, #1  ; subtract 1
        BNE     Loop1       ; branch to begin, exit loop
        B       Begin       ; stay in loop

        .endasmfunc

        .end
