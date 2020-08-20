void watchdogSetup(){
  noInterrupts();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1);

  WDOG_TOVALH = 0x006d; //1 second timer
  WDOG_TOVALL = 0xdd00;
  WDOG_PRESC = 0x400;
  WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
                  WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
                  WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
  interrupts();
}

void feedWatchdog() {
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}
