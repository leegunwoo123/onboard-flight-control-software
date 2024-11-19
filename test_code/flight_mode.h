#ifndef FLIGHT_MODE_H
#define FLIGHT_MODE_H

#include <Eigen/Dense>

// 비행 모드 열거형 정의
enum FlightMode {
    MANUAL,
    AUTO,
    GOTO
};

// 수동 비행 모드 함수
void manualFlight(Eigen::VectorXf& state);

// 자동 비행 모드 함수
void autoFlight(Eigen::VectorXf& state);

// 고투(Goto) 비행 모드 함수
void gotoFlight(Eigen::VectorXf& state);

#endif
