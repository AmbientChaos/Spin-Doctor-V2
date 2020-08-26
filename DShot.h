/*The MIT License (MIT)

Copyright (c) 2015 Extent

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include "DMAChannel.h"

#define DSHOT_COMMAND_LENGTH 16
#define DSHOT_BUFFER_LENGTH DSHOT_COMMAND_LENGTH+1

#define FTM_PINCFG(pin) FTM_PINCFG2(pin)
#define FTM_PINCFG2(pin) CORE_PIN ## pin ## _CONFIG

//FTM0_CH2 and //FTM0_CH3
#define ESC_1_PIN  9
#define ESC_2_PIN  10

#define FTM_SC_PRESCALE1      0x00
#define FTM_SC_PRESCALE2      0x01
#define FTM_SC_PRESCALE4      0x02
#define FTM_SC_PRESCALE8      0x03
#define FTM_SC_PRESCALE16     0x04
#define FTM_SC_PRESCALE32     0x05
#define FTM_SC_PRESCALE64     0x06
#define FTM_SC_PRESCALE128    0x07

#define FTM_SC_CLK_NONE       0x00
#define FTM_SC_CLK_SYS        0x08
#define FTM_SC_CLK_FIXED      0x10
#define FTM_SC_CLK_EXT        0x18

#define FTM_SC_COUNT_UP       0x00
#define FTM_SC_COUNT_UPDOWN   0x20

#define FTM_SC_IRQ            0x40

#define FTM_SC_OVERFLOW_FLAG  0x80

#define FTM_CSC_DMA           0x01

#define FTM_CSC_PWM_EDGE_HI   0x28
#define FTM_CSC_PWM_EDGE_LO   0x24

#define FTM_CSC_IRQ           0x40
#define FTM_CSC_OVERFLOW_FLAG 0x80

// clock in hz
#define DSHOT_CLOCK 600000
// bit timing as a % of clock rate
#define DSHOT_0_TIMING  (int)(0.38*255)
#define DSHOT_1_TIMING  (int)(0.75*255)

DMAChannel dma;

uint8_t dshotCommandBuffer[DSHOT_BUFFER_LENGTH];
const uint32_t mod = (F_BUS + DSHOT_CLOCK / 2) / DSHOT_CLOCK;

void setupDshotDMA(void){
  //for  (__MK64FX512__) (__MK66FX1M0__)
  FTM0_SC = 0; // disable timer
  FTM0_CNT = 0; // reset counter
  FTM0_MOD = mod - 1; // set trigger length
  FTM0_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);  //set clock source 1 and prescale 0
  FTM0_C1SC = FTM_CSC_DMA | FTM_CSC_PWM_EDGE_HI | FTM_CSC_IRQ;
  FTM0_C2SC = FTM_CSC_PWM_EDGE_HI;
  FTM0_C3SC = FTM_CSC_PWM_EDGE_HI;
  FTM0_C1V = (mod * 250) >> 8; // channel 1 drives the DMA transfer, trigger right at the end of each pulse
  FTM0_C2V = 0; //ESC 1 channel
  FTM0_C3V = 0; //ESC 2 channel

  dma.sourceBuffer((uint8_t *)dshotCommandBuffer, DSHOT_BUFFER_LENGTH);
  dma.destination(FTM0_C2V); //feed to channel 2 PWM length
  dma.transferSize(1);
  dma.transferCount(DSHOT_BUFFER_LENGTH);
  dma.disableOnCompletion();

  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM0_CH1);

  FTM_PINCFG(ESC_1_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Configure pin for ESC 1 channel in alternate mode 4 - FTM0 channel 2
  FTM_PINCFG(ESC_2_PIN) = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_SRE; //Configure pin for ESC 2 channel in alternate mode 4 - FTM0 channel 3
}

uint8_t getDshotChecksum(uint16_t value){
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 3; i++) {
    checksum ^=  value;
    value >>= 4;
  }
  checksum &= 0xf; // mask off the first nibble

  return checksum;
}

void fillDshotBuffer(uint16_t value){
  //memset(dshotCommandBuffer, 0, DSHOT_BUFFER_LENGTH); //Empty DMA buffer
  for(int i=0; i<DSHOT_COMMAND_LENGTH; i++){ // scan all the bits in the packet
    dshotCommandBuffer[15-i] = (bool)((1<<i)&value) ? (mod * DSHOT_1_TIMING) >> 8 : (mod * DSHOT_0_TIMING) >> 8; // pack buffer MSB first
  }
}

void dshotOut(uint16_t value, uint8_t motor = 1, bool telem = false){
  uint16_t packet = 0;
  uint8_t checksum = 0;

  if(motor == 1) {dma.destination(FTM0_C2V);} //set destination as ESC 1
  else if(motor == 2) {dma.destination(FTM0_C3V);} //set destination as ESC 2
  else {return;}
  dma.transferSize(1);

  if(value == 0) {} //allow "0" to be sent to shut motors off
  else if (value < 48) {value = 48;} //limit values to 48-2047 ESC range
  else if (value > 2047) {value = 2047;}

  packet = value << 1; //add telemetry bit (0)
  if (value != 0) packet = packet | telem; //set telemetry bit
  checksum = getDshotChecksum(packet);
  packet = (packet<<4)|checksum;
  fillDshotBuffer(packet);
  dma.enable();
}
