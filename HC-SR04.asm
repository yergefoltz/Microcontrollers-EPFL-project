; file    HC-SR04.asm target  ATmega128L - 4 MHz - STK300
; HC-SR04 ultrasonic distance sensor logic + beeper logic
;
;DISTANCE TIMING (why the divisor is 33)
;   Speed of sound = 343 m/s, so a round trip of 1 cm takes 58.14 us.
;   At 4 MHz one CPU cycle is 0.25 us.
;   The echo-counting loop takes 7 cycles per pass = 1.75 us.
;   Therefore 1 cm corresponds to 58.14 / 1.75 = 33.22 loop passes.
;   Dividing the raw loop count by 33 gives exact results across the full range.

; PINs
.equ TRIG = 0           ; PD0
.equ ECHO = 1           ; PD1

; distances
.equ DIST_NEAR   = 10   ; below this: very fast beeping
.equ DIST_MID    = 25   ; below this: fast beeping
.equ DIST_FAR    = 50   ; below this: slow beeping; above: silence
.equ DIST_NONE   = 0xFF ; no echo / out of range

; count to centimeter divisor
; The HC_count loop is 7 cycles (1.75 us per pass):
;   inc r21 (1) + brne taken (2) + sbis skip-1-word (2) + rjmp HC_count (2) = 7
; Round trip for 1 cm at 343 m/s = 58.14 us.
; Passes per cm = 58.14 / 1.75 = 33.22 -> divisor 33.
.equ CM_DIVISOR = 33


HC_init:
    sbi     DDRD, TRIG          ; TRIG is an output
    cbi     DDRD, ECHO          ; ECHO is an input
    cbi     PORTD, TRIG         ; TRIG idle low
    sbi     DDRE, SPEAKER       ; SPEAKER is an output
    cbi     PORTE, SPEAKER      ; buzzer off
    ret


; perform 1 distance measurement
HC_measure:
    push    r20
    push    r22
    push    r23
    push    r24
    push    r25

    ; emit 10 us (40 cycles) trigger pulse
    ; required by the sensor
    sbi     PORTD, TRIG
    ldi     r25, 10
HC_trig_delay: ; waits 40 cycles (10 us)
    nop
    nop
    dec     r25
    brne    HC_trig_delay
    cbi     PORTD, TRIG

    ; wait for echo to go high with ~30 ms timeout
    ; 120000 cycles = 30 ms
    ldi     r22, 240
    ldi     r23, 250
HC_wait_high:
    sbic    PIND, ECHO
    rjmp    HC_echo_started
    dec     r23
    brne    HC_wait_high
    ldi     r23, 250
    dec     r22
    brne    HC_wait_high
    rjmp    HC_timeout          ; timeout if echo never rises

HC_echo_started:
    ; measure how long echo stays high
    clr     r20
    clr     r21
HC_count:
    inc     r21                 ; 1 cycle
    brne    HC_count2           ; 1/2 cycles
    inc     r20                 ; carry into high byte when r21 wraps
HC_count2:
    sbis    PIND, ECHO          ; keep counting for high echo
    rjmp    HC_done             ; echo finished
    rjmp    HC_count

HC_done:
    ; r20:r21 = raw loop count. Convert to centimetres.
    ldi     r22, CM_DIVISOR
    rcall   DIV16               ; r20:r21 = count / CM_DIVISOR

    ; fit the result to <= 254 for 0xff to be reserved for timeout
    tst     r20
    brne    HC_overflow         ; high byte set -> result too large
    cpi     r21, DIST_NONE
    brne    HC_finish
HC_overflow:
    ldi     r21, 0xFE           ; 254 is far but still valid
    rjmp    HC_finish

HC_timeout:
    ldi     r21, DIST_NONE      ; 0xFF = no echo / out of range
    clr     r20

HC_finish:
    pop     r25
    pop     r24
    pop     r23
    pop     r22
    pop     r20
    ret

; unsigned 16-bit / 8-bit division
DIV16:
    clr     r23
    ldi     r24, 16             
DIV16_loop:
    lsl     r21                 
    rol     r20
    rol     r23                 
    cp      r23, r22
    brlo    DIV16_skip
    sub     r23, r22           
    inc     r21                 
DIV16_skip:
    dec     r24
    brne    DIV16_loop
    ret


; park_tone - 1 kHz beep
; 60 square-wave periods of 1 ms (0.5 ms high + 0.5 ms low)
Park_tone:
    push    r24
    push    r25
    ldi     r24, 60
Park_tone_loop:
    sbi     PORTE, SPEAKER
    WAIT_US 500
    cbi     PORTE, SPEAKER
    WAIT_US 500
    dec     r24
    brne    Park_tone_loop
    pop     r25
    pop     r24
    ret


Radar_tone:
    push    r24
    push    r25
    push    r27
    ldi     r24, 20             
    ldi     r25, 200            
Radar_tone_loop:
    sbi     PORTE, SPEAKER
    mov     r27, r25
Radar_delay1:
    nop
    dec     r27
    brne    Radar_delay1
    cbi     PORTE, SPEAKER
    mov     r27, r25
Radar_delay2:
    nop
    dec     r27
    brne    Radar_delay2
    dec     r25                 
    dec     r24
    brne    Radar_tone_loop
    pop     r27
    pop     r25
    pop     r24
    ret

