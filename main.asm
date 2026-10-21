; file    main.asm target  ATmega128L - 4 MHz - STK300 (KAN-MEG-DEV)
;   
; Top-level program. Owns shared SRAM variables, reset/init, main loop and three modes.
;
; Operating modes (variable "mode"):
;     0 = PARK   : servo stays at rest, buzzer beeps (parktronic sound)
;     1 = RADAR  : servo sweeps left<->right, buzzer beeps (radar sound)
;     2 = MANUAL : VOL+/VOL-/UP/DOWN change speed or direction of the rotation, 
;                                    make a small rotation (clockwise/counter clock.)
;
; IR télécomande (ir_rc5.asm):
;     VOL+, VOL-, UP, DOWN  -> manual mode
;     GUIDE -> change modes (PARK->RADAR->PARK, MANUAL->PARK)
;     POWER -> power on/off
;     MUTE  -> buzzer on/off


.include "macros.asm"
.include "definitions.asm"


; main passes between beeps for different distance 
.equ BUZZ_REPEAT_VFAST = 1      ; < 10 cm  : ~1 pass  (~50 ms)
.equ BUZZ_REPEAT_FAST  = 4      ; 10-24 cm : ~4 passes (~200 ms)
.equ BUZZ_REPEAT_SLOW  = 12     ; 25-49 cm : ~12 passes (~600 ms)

; glitch filter. Sudden large changes in distance are considered as glitches
.equ DIST_JUMP_LIMIT   = 20

; radar sweep timing
; Target: 4.5 seconds per full turn (30 steps). Each step = SWEEP_STEP_MS +
; HC_measure (~3 ms with echo, ~45 ms on timeout) + lcd (~2 ms).
; With an echo the total is ~153 ms; on timeout ~195 ms. The variation
; comes from HC_measure and is acceptable for a visual sweep.
.equ SWEEP_STEPS   = 15
.equ SWEEP_STEP_MS = 148

.equ PWR_ON      = 0
.equ PWR_OFF     = 1
.equ PWR_STARTUP = 2

; delays
.equ LOOP_DELAY_MS   = 50
.equ POWEROFF_DELAY_MS = 200
.equ STARTUP_DELAY_MS  = 1500


.dseg
.org 0x0200
mode:           .byte 1     ; 0=PARK, 1=RADAR, 2=MANUAL
servo_level:    .byte 1     ; 0=centre, 1..5=right, 255..251=left
last_cmd:       .byte 1     ; last received IR command
power_state:    .byte 1     ; 0=on, 1=off, 2=turning_on
buzzer_on:      .byte 1     ; 1=buzzer enabled, 0=muted
deb_counter:    .byte 1     ; IR debounce: INT7 disabled while > 0
step_timer:     .byte 1     ; UP/DOWN kick countdown (main loop decrements)
distance_cm:    .byte 1     ; raw measured distance
dist_prev:      .byte 1     ; previous raw distance (for the glitch filter)
dist_display:   .byte 1     ; filtered distance (used for LCD and beeps)
buzz_counter:   .byte 1     ; non-blocking beep cadence counter
tone_fn:        .byte 2     ; word address of the active tone routine (Park_tone or Radar_tone)


.cseg
.org 0x0000
    rjmp    reset

.org 0x0012
    rjmp    INT7_handler        ; defined in ir_rc5.asm

; peripheriques
.include "lcd.asm"
.include "printf.asm"
.include "HC-SR04.asm"
.include "servo.asm"
.include "ir_rc5.asm"


reset:
    LDSP        RAMEND
    rcall       LCD_init
    rcall       HC_init
    rcall       Servo_init

    ldi         r16, 0
    sts         mode, r16 ; set to parking by default
    sts         servo_level, r16
    sts         power_state, r16 ; set to power on
    sts         deb_counter, r16
    sts         step_timer, r16
    sts         distance_cm, r16
    sts         dist_prev, r16
    sts         dist_display, r16
    sts         buzz_counter, r16
    ldi         r16, 1
    sts         buzzer_on, r16  ; buzzer enabled by default
    ldi         r16, 0xFF
    sts         last_cmd, r16

    ; default - park tone, switched to radar mode - radar tone
    ldi         r16, low(Park_tone)
    sts         tone_fn, r16
    ldi         r16, high(Park_tone)
    sts         tone_fn+1, r16

    ldi         r16, 0
    rcall       Servo_set_level ; set servo at centre

    ; enable INT7 on the falling edge (button pressed)
    ldi         r16, (1<<ISC71)|(0<<ISC70)
    sts         EICRB, r16
    ldi         r16, (1<<INT7)
    out         EIMSK, r16
    sei

    rjmp        main


main:
    ; When step_timer counts down to 0, stop the rotation
    lds         r16, step_timer
    cpi         r16, 0
    breq        main_no_step
    dec         r16
    sts         step_timer, r16
    cpi         r16, 0
    brne        main_no_step
    ldi         r16, 0
    sts         servo_level, r16

main_no_step:
    ; countdown to enable remote again if the button was pushed
    rcall       deb_tick

    ; power on/off logic
    lds         r16, power_state
    cpi         r16, PWR_OFF
    brne        PC+2
    rjmp        main_power_off
    cpi         r16, PWR_STARTUP
    brne        PC+2
    rjmp        main_power_on_seq

    ; mode RADAR
    lds         r16, mode
    cpi         r16, 1
    brne        PC+2
    rjmp        main_radar

    ; mode PARK/MANUAL
    lds         r16, servo_level
    rcall       Servo_set_level

    ; measure distance
    rcall       measure_and_filter

    rcall       buzz_tick

main_lcd:
    rcall       lcd_show
    WAIT_MS     LOOP_DELAY_MS
    rjmp        main

; displays POWER OFF, buttons disabled, no sound, no rotation
main_power_off:
    ldi         r16, 0
    rcall       Servo_set_level
    cbi         PORTE, SPEAKER

    rcall       LCD_home
    PRINTF      LCD
    .db "POWER OFF       ", 0
    ldi         r18, 0x40
    rcall       LCD_pos
    PRINTF      LCD
    .db "                ", 0

    WAIT_MS     POWEROFF_DELAY_MS
    rjmp        main

; power was justed turned on, wait for STARTUP_DELAY_MS for display
main_power_on_seq:
    rcall       LCD_home
    PRINTF      LCD
    .db "POWER ON        ", 0
    ldi         r18, 0x40
    rcall       LCD_pos
    PRINTF      LCD
    .db "  Starting...   ", 0

    WAIT_MS     STARTUP_DELAY_MS

    ; re-initialise state (same defaults as reset, without the one-time setup)
    ldi         r16, 0
    sts         mode, r16
    sts         servo_level, r16
    sts         power_state, r16
    sts         step_timer, r16
    sts         distance_cm, r16
    sts         dist_prev, r16
    sts         dist_display, r16
    sts         buzz_counter, r16
    ldi         r16, 1
    sts         buzzer_on, r16
    ldi         r16, low(Park_tone)
    sts         tone_fn, r16
    ldi         r16, high(Park_tone)
    sts         tone_fn+1, r16
    ldi         r16, 0
    rcall       Servo_set_level

    rjmp        main


; mode RADAR logic
main_radar:
    ; switch the tone of the buzzer
    ldi         r16, low(Radar_tone)
    sts         tone_fn, r16
    ldi         r16, high(Radar_tone)
    sts         tone_fn+1, r16


    rcall       Servo_level_half ; set direction to right
    ldi         r18, SWEEP_STEPS

main_radar_fwd:
    push        r18

    ; same as PARK logic 
    rcall       measure_and_filter
    rcall       buzz_tick               
    rcall       lcd_show
    rcall       deb_tick
    rcall       sweep_dwell ; wait on the current posision for SWEEP_STEP_MS ms

    pop         r18

    lds         r16, mode
    cpi         r16, 1
    brne        main_radar_exit
    dec         r18
    brne        main_radar_fwd

    rcall       Servo_level_mhalf       ; change the direction
    ldi         r18, SWEEP_STEPS

main_radar_bwd:
    push        r18

    rcall       measure_and_filter
    rcall       buzz_tick
    rcall       lcd_show
    rcall       deb_tick
    rcall       sweep_dwell ; wait on the current posision for SWEEP_STEP_MS ms

    pop         r18

    lds         r16, mode
    cpi         r16, 1
    brne        main_radar_exit
    dec         r18
    brne        main_radar_bwd

    rjmp        main_radar

main_radar_exit:
    ; restore the state of the system to PARK/MANUAL mode
    ldi         r16, low(Park_tone)
    sts         tone_fn, r16
    ldi         r16, high(Park_tone)
    sts         tone_fn+1, r16
    ldi         r16, 0
    sts         servo_level, r16
    rcall       Servo_set_level
    rjmp        main_lcd


; delay for each small sweep step ( basically speed adjusting )
sweep_dwell:
    WAIT_MS     SWEEP_STEP_MS
    ret

; sound (beeping) logic
buzz_tick:

    ; muted
    lds         r16, buzzer_on
    cpi         r16, 0
    breq        bt_silent


    lds         r16, buzz_counter
    cpi         r16, 0
    brne        bt_wait

    ; checks if beeping is needed (if distance <= 50 cm)
    lds         r21, dist_display
    cpi         r21, DIST_NONE
    breq        bt_silent               ; no echo -> silence
    cpi         r21, DIST_FAR
    brsh        bt_silent               ; >= 50 cm -> silence

    ; play one beep with tone corresponding to the mode
    lds         zl, tone_fn
    lds         zh, tone_fn+1
    icall

    ; reload rhythme counter based on current distance.
    lds         r21, dist_display
    cpi         r21, DIST_NEAR
    brsh        bt_check25
    ldi         r16, BUZZ_REPEAT_VFAST  ; < 10 cm
    rjmp        bt_set

bt_check25:
    cpi         r21, DIST_MID
    brsh        bt_slow
    ldi         r16, BUZZ_REPEAT_FAST   ; 10-24 cm
    rjmp        bt_set
    
bt_slow:
    ldi         r16, BUZZ_REPEAT_SLOW   ; 25-49 cm

bt_set:
    sts         buzz_counter, r16
    ret

bt_wait:
    dec         r16
    sts         buzz_counter, r16
    ret

bt_silent:
    cbi         PORTE, SPEAKER
    ldi         r16, 0
    sts         buzz_counter, r16
    ret


; read ultrasonic sensor logic
measure_and_filter:
    push        r16
    push        r17

    rcall       HC_measure              ; r21 = distance in cm
    sts         distance_cm, r21

    ; r16 = previous sample then remember the new raw sample.
    lds         r16, dist_prev
    sts         dist_prev, r21

    ; check if the previous or current distance is available
    cpi         r21, DIST_NONE
    breq        mf_accept               
    cpi         r16, DIST_NONE
    breq        mf_accept               

    ; r17 = |new - prev|
    mov         r17, r21
    sub         r17, r16
    brpl        mf_pos
    neg         r17

mf_pos:
    cpi         r17, DIST_JUMP_LIMIT
    brlo        mf_accept               ; small change -> accept
    ; large jump -> treat as a glitch, keep the previous value.
    mov         r21, r16

mf_accept:
    sts         dist_display, r21

    pop         r17
    pop         r16
    ret


; line 1: mode + speed level
; line 2: distance in cm or --- when no echo
lcd_show:
    push        r20
    push        r18

    ; line 1
    rcall       LCD_home
    lds         r16, mode

    cpi         r16, 2
    brne        lcd_chk_radar

    ; MANUAL: show the servo speed level
    lds         r20, servo_level
    PRINTF      LCD
    .db "Manual  s=", FDEC|FSIGN, 20, "   ", 0
    rjmp        lcd_line2

lcd_chk_radar:
    cpi         r16, 1
    brne        lcd_park

    PRINTF      LCD
    .db "Radar   s= 0   ", 0
    rjmp        lcd_line2

lcd_park:
    PRINTF      LCD
    .db "Park    s= 0   ", 0

lcd_line2:
    ; line 2
    ldi         r18, 0x40
    rcall       LCD_pos

    lds         r20, dist_display
    cpi         r20, DIST_NONE
    brne        lcd_dist_ok

    PRINTF      LCD
    .db "Dist: --- cm    ", 0
    rjmp        lcd_end

lcd_dist_ok:
    PRINTF      LCD
    .db "Dist: ", FDEC|FDIG3, 20, " cm     ", 0

lcd_end:
    pop         r18
    pop         r20
    ret
