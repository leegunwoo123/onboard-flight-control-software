#include "pose_estimator.h"
#include "flight_mode.h"
#include "flight_control.h"
#include <thread>
#include <iostream>
#include <fstream>
#include <iomanip>

int main() {
    // 루프 실행 주기 설정 (100ms)
    const std::chrono::milliseconds loopDuration(100);

        // 비행 제어 시스템 초기화 (RC, GPS, IMU 등)
    flight_control_init();

    // EKF 기반 자세 추정 클래스 생성
    PoseEstimator poseEstimator;
    std::this_thread::sleep_for(loopDuration);
    

    // CSV 파일 열기
    std::ofstream csvFile("current_pose.csv");
    if (!csvFile.is_open()) {
        std::cerr << "CSV 파일을 열 수 없습니다." << std::endl;
        return 1;  // 파일 열기 실패 시 프로그램 종료
    }

    // CSV 파일 헤더 작성
    csvFile << "X,Y,Z,Roll,Pitch,Yaw" << std::endl;

    // 메인 루프
    while (true) {
        // 100ms 주기로 상태 값을 가져옴
        Eigen::VectorXf state = poseEstimator.getPose();

        // 자세 추정값 출력 (x, y, z 위치와 roll, pitch, yaw만 출력)
        std::cout << std::fixed << std::setprecision(7); // 소수점 7자리까지 표시
        std::cout << "Current Pose: "
                  << state(0) << " "
                  << state(1) << " "
                  << state(2) << " "
                  << state(6) << " "
                  << state(7) << " "
                  << state(8) << std::endl;

        // 100ms 동안 대기
        std::this_thread::sleep_for(loopDuration);
    }

    // CSV 파일 닫기
    csvFile.close();

    return 0;
}
