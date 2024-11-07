#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <string>

// RC 입력 초기화 함수
void initRC(const std::string& port, int baudRate);

// RC 데이터를 읽는 함수
int readRCChannel(int channel);

#endif // RC_INPUT_H