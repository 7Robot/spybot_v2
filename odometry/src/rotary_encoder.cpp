#include <iostream>

#include <pigpiod_if2.h>

#include "rotary_encoder.hpp"

/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/

void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
   if (gpio == mygpioA) levA = level; else levB = level;

   if (gpio != lastGpio) /* debounce */
   {
      lastGpio = gpio;

      if ((gpio == mygpioA) && (level == 1))
      {
         if (levB) (mycallback)(1);
      }
      else if ((gpio == mygpioB) && (level == 1))
      {
         if (levA) (mycallback)(-1);
      }
   }
}

void re_decoder::_pulseEx(int PI, unsigned int gpio, unsigned int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   re_decoder *mySelf = (re_decoder *) user;

   mySelf->_pulse(gpio, level, tick); /* Call the instance callback. */
}

re_decoder::re_decoder(int PI, int gpioA, int gpioB, re_decoderCB_t callback)
{
   m_PI = PI;
   mygpioA = gpioA;
   mygpioB = gpioB;

   mycallback = callback;

   levA=0;
   levB=0;

   lastGpio = -1;

   set_mode(m_PI, gpioA, PI_INPUT);
   set_mode(m_PI, gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   set_pull_up_down(PI, gpioA, PI_PUD_UP);
   set_pull_up_down(PI, gpioB, PI_PUD_UP);

   /* monitor encoder level changes */

   m_id_A = callback_ex(PI, gpioA, EITHER_EDGE, _pulseEx, this);
   m_id_B = callback_ex(PI, gpioB, EITHER_EDGE, _pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   callback_cancel(m_id_A);
   callback_cancel(m_id_B);
}
