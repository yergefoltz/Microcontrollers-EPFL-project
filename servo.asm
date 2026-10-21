; file    servo.asm target  ATmega128L - 4 MHz - STK300
; SG-90 servo-motor logic 


; Timer3 OC3A = PE3
.equ SERVO_PIN = 3

; pulse widths (OCR3A counts) for each level 
; the PWM period (ICR3) is 10000 counts; these values are the servo
; pulse lengths that produce each steering position.
.equ OCR_CENTRE = 760
.equ OCR_R1     = 765
.equ OCR_R2     = 770
.equ OCR_R3     = 775
.equ OCR_R4     = 780
.equ OCR_R5     = 785
.equ OCR_L1     = 750
.equ OCR_L2     = 745
.equ OCR_L3     = 740
.equ OCR_L4     = 735
.equ OCR_L5     = 725

; sweep end-points used by the radar mode
.equ OCR_SWEEP_R = 775
.equ OCR_SWEEP_L = 735

.equ SERVO_PERIOD = 10000   ; Timer3 TOP (ICR3) = PWM period


; load a 16-bit OCR value into r18:r17 (high:low) and return.
.macro  SET_OCR             ; @0 = OCR constant
    ldi     r17, low(@0)
    ldi     r18, high(@0)
    ret
    .endmacro


Servo_init:

    ; setup Timer3 to generate a PWM signal on the servo pin
    sbi     DDRE, SERVO_PIN
    ldi     r16, (1<<COM3A1)|(1<<WGM31)
    sts     TCCR3A, r16
    ldi     r16, (1<<WGM33)|(1<<WGM32)|(1<<CS31)
    sts     TCCR3B, r16

    ldi     r16, high(SERVO_PERIOD)
    sts     ICR3H, r16
    ldi     r16, low(SERVO_PERIOD)
    sts     ICR3L, r16

    cli
    ldi     r17, high(OCR_CENTRE)
    sts     OCR3AH, r17
    ldi     r17, low(OCR_CENTRE)
    sts     OCR3AL, r17
    sei
    ret

; sets speed based on OCR value
Servo_set_level:
    push    r17
    push    r18
    rcall   Servo_get_ocr       ; r18:r17 = OCR value for this level
    cli
    sts     OCR3AH, r18
    sts     OCR3AL, r17
    sei
    pop     r18
    pop     r17
    ret

; servo_set_level without disabling interruptions
Servo_set_level_raw:
    push    r17
    push    r18
    rcall   Servo_get_ocr
    sts     OCR3AH, r18
    sts     OCR3AL, r17
    pop     r18
    pop     r17
    ret

; sets OCR value based on the level
Servo_get_ocr:
    cpi     r16, 0
    brne    SG_1
    SET_OCR OCR_CENTRE
SG_1:
    cpi     r16, 1
    brne    SG_2
    SET_OCR OCR_R1
SG_2:
    cpi     r16, 2
    brne    SG_3
    SET_OCR OCR_R2
SG_3:
    cpi     r16, 3
    brne    SG_4
    SET_OCR OCR_R3
SG_4:
    cpi     r16, 4
    brne    SG_5
    SET_OCR OCR_R4
SG_5:
    cpi     r16, 5
    brne    SG_m1
    SET_OCR OCR_R5
SG_m1:
    cpi     r16, 255
    brne    SG_m2
    SET_OCR OCR_L1
SG_m2:
    cpi     r16, 254
    brne    SG_m3
    SET_OCR OCR_L2
SG_m3:
    cpi     r16, 253
    brne    SG_m4
    SET_OCR OCR_L3
SG_m4:
    cpi     r16, 252
    brne    SG_m5
    SET_OCR OCR_L4
SG_m5:
    cpi     r16, 251
    brne    SG_default
    SET_OCR OCR_L5
SG_default:
    SET_OCR OCR_CENTRE          ; unknown level -> centre

; set servo to rotate right continuously
Servo_level_half:
    cli
    ldi     r17, high(OCR_SWEEP_R)
    sts     OCR3AH, r17
    ldi     r17, low(OCR_SWEEP_R)
    sts     OCR3AL, r17
    sei
    ret


; set servo to rotate left continuously
Servo_level_mhalf:
    cli
    ldi     r17, high(OCR_SWEEP_L)
    sts     OCR3AH, r17
    ldi     r17, low(OCR_SWEEP_L)
    sts     OCR3AL, r17
    sei
    ret
