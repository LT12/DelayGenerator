//
// Created by Larry on 2/8/2017.
//

#ifndef DELAYGENERATOR_DELAYGENERATOR_H
#define DELAYGENERATOR_DELAYGENERATOR_H


bool looping;
size_t seqlen, startcounter, endcounter;
uint32_t pulseregstart[50];
uint32_t starttimes[50];
uint32_t endtimes[50][2];

void playSeq(void);
void loadSeq(void);
void clearSeq(void);


#endif //DELAYGENERATOR_DELAYGENERATOR_H
