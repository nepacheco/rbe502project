/* 
    Author: Greg Lewin
 */
#ifndef __EVENT_TIMER_H
#define __EVENT_TIMER_H
#include "mbed.h"

class EventTimer
{
  protected:
    uint8_t timerState = 0; // 1 if timer running else 0
    uint32_t startTime = 0; // in ms
    uint32_t duration = 0;  // in ms
    
  public:
    EventTimer(void){}
    
    bool CheckExpired(void){
        bool retVal = false;
        if(millis() - startTime > duration && timerState == 1)
        {
            timerState = 0;
            retVal = true;
        }
        
        return retVal;
    }
    
    unsigned long Start(unsigned long dur){
      startTime = millis();
      duration = dur;
      timerState = 1;
      
      return startTime + dur;
    }

     //need a better name?
    unsigned long Restart(unsigned long dur = 0){
      if(dur != 0) duration = dur;
      startTime += duration;
      timerState = 1;
      
      return startTime + duration;
    }
    
    void Cancel(void){
      timerState = 0;
    }
    
    int IsRunning(void){
        return timerState;
    }
};

#endif