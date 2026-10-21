; === IR RC-5 Receiver Driver ===
; file    ir_rc5.asm target  ATmega128L-4MHz-STK300
; IR RC-5 Receiver logic

.equ T1         = 1778      ; RC-5 bit period in microseconds

; IR command codes
.equ CMD_VOLUP  = 0xE0      ; VOL+  
.equ CMD_VOLDN  = 0xE1      ; VOL-  
.equ CMD_UP     = 0xC0      ; UP   
.equ CMD_DOWN   = 0xC1      ; DOWN  
.equ CMD_GUIDE  = 0xC2      ; GUIDE
.equ CMD_POWER  = 0xEC      ; POWER
.equ CMD_MUTE   = 0xED      ; MUTE

.equ STEP_KICK_PASSES = 12


; deb_tick - debounce tick, call once per main loop iteration
; decrements deb_counter; re-enables INT7 when it reaches 0.
deb_tick:
    lds         r16, deb_counter
    cpi         r16, 0
    breq        deb_tick_done
    dec         r16
    sts         deb_counter, r16
    cpi         r16, 0
    brne        deb_tick_done
    ldi         r16, (1<<INT7)
    out         EIMSK, r16
deb_tick_done:
    ret


; INT7_handler - IR interrupt service routine
; Decodes one RC-5 command sent
INT7_handler:
    push        r16
    push        r17
    push        r18
    push        r20
    push        r22
    push        r23
    push        r24
    push        r3
    in          r16, SREG
    push        r16

    ; decode 14-bit RC-5 frame
    ; wait T1/4 to align the first sample to the middle of the start bit,
    ; then sample one bit every T1 us. Each bit is shifted into b1:b0.
    CLR2        b1, b0
    ldi         b2, 14
    WAIT_US     (T1/4)

INT7_loop:
    P2C         PINE, IR            ; sample IR pin -> Carry
    ROL2        b1, b0              ; shift Carry into b1:b0 from LSB
    WAIT_US     (T1-4)              ; wait one bit period (minus loop overhead)
    DJNZ        b2, INT7_loop       ; repeat for all 14 bits

    sts         last_cmd, b0        ; save command byte (low 8 bits of frame)

    cpi         b0, CMD_POWER
    brne        PC+2
    rjmp        INT7_power

    ; all other commands are ignored while powered off
    lds         r16, power_state
    cpi         r16, 0
    brne        PC+2
    rjmp        INT7_dispatch
    rjmp        INT7_end

INT7_dispatch:
    cpi         b0, CMD_MUTE
    brne        PC+2
    rjmp        INT7_mute
    cpi         b0, CMD_VOLUP
    brne        PC+2
    rjmp        INT7_vol_up
    cpi         b0, CMD_VOLDN
    brne        PC+2
    rjmp        INT7_vol_dn
    cpi         b0, CMD_UP
    brne        PC+2
    rjmp        INT7_step_fwd
    cpi         b0, CMD_DOWN
    brne        PC+2
    rjmp        INT7_step_bwd
    cpi         b0, CMD_GUIDE
    brne        PC+2
    rjmp        INT7_guide
    rjmp        INT7_end


; disables INT7 with deb_counter=5 to avoid bounce/repeat.
INT7_power:
    ldi         r16, 0
    out         EIMSK, r16
    ldi         r16, 5
    sts         deb_counter, r16

    lds         r16, power_state
    cpi         r16, 0
    breq        INT7_power_off
   
    ldi         r16, 2
    sts         power_state, r16
    rjmp        INT7_end
INT7_power_off:

    ldi         r16, 1
    sts         power_state, r16
    rjmp        INT7_end

; mute
INT7_mute:
    ldi         r16, 0
    out         EIMSK, r16
    ldi         r16, 3
    sts         deb_counter, r16

    lds         r16, buzzer_on
    cpi         r16, 1
    breq        INT7_mute_off
    ldi         r16, 1
    sts         buzzer_on, r16
    rjmp        INT7_end
INT7_mute_off:
    ldi         r16, 0
    sts         buzzer_on, r16
    rjmp        INT7_end

; vol+ = increase/decrease speed (depends on the direction of the rotation)
INT7_vol_up:
    ldi         r16, 2
    sts         mode, r16
    ldi         r16, 0
    sts         step_timer, r16
    lds         r17, servo_level
    cpi         r17, 251            
    brlo        vup_not_neg
    inc         r17                 
    rjmp        vup_apply
vup_not_neg:
    cpi         r17, 5              
    brlo        PC+2
    rjmp        INT7_end            
    inc         r17
vup_apply:
    sts         servo_level, r17
    mov         r16, r17
    rcall       Servo_set_level_raw
    ldi         r16, 0
    out         EIMSK, r16
    ldi         r16, 3
    sts         deb_counter, r16
    rjmp        INT7_end

; vol- = decrease/increase speed (depends on the direction of the rotation)
INT7_vol_dn:
    ldi         r16, 2
    sts         mode, r16
    ldi         r16, 0
    sts         step_timer, r16
    lds         r17, servo_level
    cpi         r17, 6              
    brsh        vdn_not_pos
    cpi         r17, 0
    breq        vdn_from_zero
    dec         r17
    rjmp        vdn_apply
vdn_from_zero:
    ldi         r17, 255            
    rjmp        vdn_apply
vdn_not_pos:
    cpi         r17, 251            
    brne        PC+2
    rjmp        INT7_end           
    dec         r17
vdn_apply:
    sts         servo_level, r17
    mov         r16, r17
    rcall       Servo_set_level_raw
    ldi         r16, 0
    out         EIMSK, r16
    ldi         r16, 3
    sts         deb_counter, r16
    rjmp        INT7_end

; small rotation in counter-clockwise direction
INT7_step_fwd:
    ldi         r16, 2
    sts         mode, r16
    ldi         r17, 1
    sts         servo_level, r17
    ldi         r17, STEP_KICK_PASSES
    sts         step_timer, r17
    rjmp        INT7_end

; small rotation in clockwise direction
INT7_step_bwd:
    ldi         r16, 2
    sts         mode, r16
    ldi         r17, 255
    sts         servo_level, r17
    ldi         r17, STEP_KICK_PASSES
    sts         step_timer, r17
    rjmp        INT7_end


; change modes MANUAL -> PARK, RADAR -> PARK, PARK -> RADAR
INT7_guide:
    ldi         r16, 0
    out         EIMSK, r16
    ldi         r16, 5
    sts         deb_counter, r16

    lds         r16, mode
    cpi         r16, 2              ; MANUAL
    breq        guide_to_park
    cpi         r16, 1              ; RADAR
    breq        guide_to_park
    ; PARK -> RADAR
    ldi         r16, 1
    sts         mode, r16
    rjmp        INT7_end
guide_to_park:
    ldi         r16, 0
    sts         mode, r16
    sts         servo_level, r16
    sts         step_timer, r16
    rcall       Servo_set_level_raw
    rjmp        INT7_end

INT7_end:
    pop         r16
    out         SREG, r16
    pop         r3
    pop         r24
    pop         r23
    pop         r22
    pop         r20
    pop         r18
    pop         r17
    pop         r16
    reti