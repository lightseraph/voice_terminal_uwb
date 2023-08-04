#define __FREQTABLE_C__
#include "config.h"

USER_DATA_DEF USER_DATA;

// A通道频点表
unsigned int FreqTableA[] = {
    PAIR_FREQ_CHA, // 索引0为对频频点
    //
    /* (START_FREQ_CHA + (FREQ_STEP * 0)),
    (START_FREQ_CHA + (FREQ_STEP * 1)),
    (START_FREQ_CHA + (FREQ_STEP * 2)),
    (START_FREQ_CHA + (FREQ_STEP * 3)),
    (START_FREQ_CHA + (FREQ_STEP * 4)),
    (START_FREQ_CHA + (FREQ_STEP * 5)),
    (START_FREQ_CHA + (FREQ_STEP * 6)),
    (START_FREQ_CHA + (FREQ_STEP * 7)),
    (START_FREQ_CHA + (FREQ_STEP * 8)),
    (START_FREQ_CHA + (FREQ_STEP * 9)), */
    6544, 6622, 6754, 6922};

// B通道频点表
unsigned int FreqTableB[] = {
    PAIR_FREQ_CHB, // 索引0为对频频点
    //
    (START_FREQ_CHB + (FREQ_STEP * 0)),
    (START_FREQ_CHB + (FREQ_STEP * 1)),
    (START_FREQ_CHB + (FREQ_STEP * 2)),
    (START_FREQ_CHB + (FREQ_STEP * 3)),
    (START_FREQ_CHB + (FREQ_STEP * 4)),
    (START_FREQ_CHB + (FREQ_STEP * 5)),
    (START_FREQ_CHB + (FREQ_STEP * 6)),
    (START_FREQ_CHB + (FREQ_STEP * 7)),
    (START_FREQ_CHB + (FREQ_STEP * 8)),
    (START_FREQ_CHB + (FREQ_STEP * 9)),
};