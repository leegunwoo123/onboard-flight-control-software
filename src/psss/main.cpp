// #include "pose_estimator.h"
// #include "flight_mode.h"
// #include "flight_control.h"
// #include <thread>
// #include <iostream>

// int main() {
//     // 비행 제어 시스템 초기화 (RC, GPS, IMU 등)
//     flight_control_init();

//     // EKF 기반 자세 추정 클래스 생성 및 주기적인 계산 시작
//     PoseEstimator poseEstimator;

//     // 현재 비행 모드를 저장하는 변수
//     FlightMode currentMode = getFlightMode();

//     // 비행 모드를 처리하는 스레드를 위한 변수
//     std::thread flightThread;

//     // 루프 실행 주기 설정 (10ms)
//     const std::chrono::milliseconds loopDuration(10);

//     // 메인 루프
//     while (true) {
//         // 현재 비행 모드 확인
//         FlightMode newMode = getFlightMode();

//         // 10ms 주기로 상태 값을 가져옴
//         Eigen::VectorXf state = poseEstimator.getPose();

//         // 새로운 비행 모드가 설정되면 스레드 처리
//         if (newMode != currentMode) {
//             if (flightThread.joinable()) {
//                 flightThread.join();  // 기존 스레드가 끝날 때까지 대기
//             }

//             // 새로운 비행 모드에 따른 스레드 생성
//             if (newMode == MANUAL) {
//                 flightThread = std::thread(manualFlight, std::ref(state));  // 수동 비행 모드
//             } else if (newMode == AUTO) {
//                 flightThread = std::thread(autoFlight, std::ref(state));  // 자동 비행 모드
//             } else if (newMode == GOTO) {
//                 flightThread = std::thread(gotoFlight, std::ref(state));  // 고투 비행 모드
//             }

//             // 현재 비행 모드를 갱신
//             currentMode = newMode;
//         }

//         // 10ms 동안 대기
//         std::this_thread::sleep_for(loopDuration);
//     }

//     // 프로그램 종료 시 스레드 종료
//     if (flightThread.joinable()) {
//         flightThread.join();
//     }

//     return 0;
// }

#include "pose_estimator.h"
#include "flight_mode.h"
#include "flight_control.h"
#include <thread>
#include <iostream>
#include <fstream>  // 파일 입출력을 위한 헤더
#include <iomanip>  // std::setprecision을 사용하기 위한 헤더

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

        // // CSV 파일에 데이터 기록
        // csvFile << std::fixed << std::setprecision(7); // 소수점 7자리까지 기록
        // csvFile << state(0) << ","
        //         << state(1) << ","
        //         << state(2) << ","
        //         << state(6) << ","
        //         << state(7) << ","
        //         << state(8) << std::endl;

        // 100ms 동안 대기
        std::this_thread::sleep_for(loopDuration);
    }

    // CSV 파일 닫기
    csvFile.close();

    return 0;
}
