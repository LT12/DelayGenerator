//
// Created by Larry on 2/8/2017.
//

#ifndef DELAYGENERATOR_DELAYGENERATOR_H
#define DELAYGENERATOR_DELAYGENERATOR_H


bool looping;
int seqlen, startcounter, endcounter;
uint32_t pulseregend[125];
uint32_t pulseregstart[125];
uint32_t starttimes[125];
uint32_t endtimes[125];
void triggerFunc(void);
void playSeq(void);
void loadSeq(void);
void clearSeq(void);
void initAutoTrig(void);

#endif //DELAYGENERATOR_DELAYGENERATOR_H
