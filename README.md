This is a fork of William B. Phelps' version of the Bulbdial code,
available from https://github.com/wbphelps/BulbdialClock

The following changes have been made:

 - Fixed annoying bug where "next" LED in each ring was always dimly lit

 - RTC calculation incorrectly used an unsigned int as a temporary. These values
   can reach 86400, which is bigger than 2^16. Switch to use a long instead.
   (I note that this bug is present in the original Bulbdial code.)

 - Reenable regular RTC updates, which were disabled, likely due to preceding
   bug.

 - Fade modes are now:
     0 - no fading
     1 - straddle 2 LEDs for odd seconds
     3 - original fading support
     4 - original fading support, move hour hand at 31 minutes
     5 - continuous fading
     6 - continuous fading, move hour hand at 31 minutes
     7 - continuous logarithmic fading
     8 - continuous logarithmic fading, move hour hand at 31 minutes

 - Removed hour hand fading; the LEDs are too far apart for it to produce 
   acceptable clock hands

 - I don't care for minute fading either so made it a compile option
   (MINUTEFADE)

 - Photocell support. This requires a board modification to swap
   LED2 from PC0/ADC0 (pin 23) to PB4/MISO (pin 18). This then
   frees up ADC0 as an analog input for a CDS photocell with a 5.6K
   pullup resistor. The photocell can then be wired to the ICSP header.
   Note that the constant values will almost certainly require adjustment
   depending on the photocell placement in the case as well as the case
   type. This support is enabled via a compile option (PHOTOCELL).

 - Brightness buttons no longer wrap.
