#define __SFR_OFFSET 0

#include <avr/io.h>

 .global setupTimer

setupTimer:

  cli
  clr r16
  sts TCCR1A,   r16
  sts TCCR1B,   r16

  clr r16
  ldi r16,      0x00
  sts TCNT1,    r16                   ; Preload timer

  clr r16
  ldi r16,      (1 << CS12)           ; Set timer to system clock / 256
  sts TCCR1B,   r16

  clr r16
  ldi r16,      (1 << TOIE1)            ; Enable overflow interrupt
  sts TIMSK1,   r16
  sei
  ret


;    TCCR1A = 0;
;  TCCR1B = 0;
;
;  TCNT1 = 34286;            // preload timer 65536-16MHz/256/2Hz
;  TCCR1B |= (1 << CS12);    // 256 prescaler 
;  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
