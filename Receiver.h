#include <CPPM-RX.h>

bool newInput = false;
uint16_t throt = 0;
uint16_t rudd = 500;
uint16_t elev = 500;
uint16_t ailer = 500;
uint16_t gear = 500;
uint16_t flap = 500;

//limit throttle pulse widths to 1000-2000us
   
int volatile return_THROT()
{
    return max(1000, min(2000, RC_THROT));
}

int volatile return_AILER()
{
    return max(1000, min(2000, RC_AILER));
}

int volatile return_ELEV()
{
    return max(1000, min(2000, RC_ELEV));
}

int volatile return_RUDD()
{
    return max(1000, min(2000, RC_RUDD));
}

int volatile return_GEAR()
{
    return max(1000, min(2000, RC_MODE));
}

int volatile return_FLAP()
{
    return max(1000, min(2000, RC_AUX1));
}

void readReceiver()
{
  noInterrupts();
  newInput = true;
  
  //get receiver inputs as a percentage of max (assuming 1000-2000 us pulse range)
  throt = (return_THROT()-1000);
  rudd = (return_RUDD()-1000);
  elev = (return_ELEV()-1000);
  ailer = (return_AILER()-1000);
  gear = (return_GEAR()-1000);
  flap = (return_FLAP()-1000);
  interrupts();
}

bool signalLost()
{
  //Bind ESC with throttle travel at absolute minimum for failsafe, below 1000us pulses
  //then return travel to normal.  On losing signal it will give an abnormal low result.
  if(RC_THROT <= 995) return true;
  else return false;
}
