// #include "ekf.h"
// #include <Eigen/Dense>
// #include <cmath>
// #include <iostream>

// // 상수
// const float GRAVITY = 9.80665f;  // 중력 상수
// const float GYRO_THRESHOLD = 0.001f;  // 자이로 변화 임계값 (필터링 강화)
// const float ALPHA = 0.9f;  // 저주파 필터 계수

// // 유틸리티 함수
// float radToDeg(float rad) {
//     return rad * (180.0f / M_PI);
// }

// float degToRad(float deg) {
//     return deg * (M_PI / 180.0f);
// }

// bool isValidValue(float value) {
//     return !std::isnan(value) && !std::isinf(value);
// }

// // 생성자
// EKF::EKF() {
//     state = Eigen::VectorXf::Zero(9);
//     covariance = Eigen::MatrixXf::Identity(9, 9) * 0.05f;

//     processNoise = Eigen::MatrixXf::Zero(9, 9);
//     processNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.1f;   // 위치 노이즈 감소
//     processNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.1f;    // 속도 노이즈 감소
//     processNoise.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 0.001f;  // 자세 노이즈 감소

//     measurementNoise = Eigen::MatrixXf::Zero(6, 6);
//     measurementNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.01f;  // GPS 위치 노이즈 감소
//     measurementNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.01f;  // GPS 속도 노이즈 감소

//     accelLast = Eigen::Vector3f::Zero();
//     gyroLast = Eigen::Vector3f::Zero();
// }

// // 소멸자
// EKF::~EKF() {}

// Eigen::Matrix3f EKF::eulerAnglesToRotationMatrix(const Eigen::Vector3f& eulerAngles) const {
//     float roll = eulerAngles(0);
//     float pitch = eulerAngles(1);
//     float yaw = eulerAngles(2);

//     Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

//     Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
//     Eigen::Matrix3f rotationMatrix = q.matrix();
//     return rotationMatrix;
// }

// // 저주파 필터 적용 함수
// Eigen::Vector3f EKF::lowPassFilter(const Eigen::Vector3f& input, const Eigen::Vector3f& last, float alpha) {
//     return alpha * last + (1.0f - alpha) * input;
// }

// // 예측 함수
// void EKF::predict(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     if (!isValidValue(accel.norm()) || !isValidValue(gyro.norm())) {
//         // std::cerr << "유효하지 않은 IMU 데이터 감지됨" << std::endl;
//         return;
//     }

//     // 저주파 필터 적용
//     Eigen::Vector3f filteredAccel = lowPassFilter(accel, accelLast, ALPHA);
//     Eigen::Vector3f filteredGyro = lowPassFilter(gyro, gyroLast, ALPHA);
    
//     if (filteredGyro.norm() < GYRO_THRESHOLD) {
//         // 자이로 변화가 작을 때 자세를 업데이트하지 않음
//         return;
//     }

//     predictState(filteredAccel, filteredGyro, dt);
//     computeJacobian(filteredAccel, filteredGyro, dt);
//     covariance = jacobian * covariance * jacobian.transpose() + processNoise;

//     // 마지막 가속도, 자이로 값 업데이트
//     accelLast = filteredAccel;
//     gyroLast = filteredGyro;
// }

// void EKF::predictState(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     Eigen::Vector3f position = state.segment<3>(0);
//     Eigen::Vector3f velocity = state.segment<3>(3);
//     Eigen::Vector3f attitude = state.segment<3>(6);

//     attitude += gyro * dt;

//     for (int i = 0; i < 3; ++i) {
//         if (attitude(i) > M_PI)
//             attitude(i) -= 2 * M_PI;
//         else if (attitude(i) < -M_PI)
//             attitude(i) += 2 * M_PI;
//     }

//     Eigen::Matrix3f rotationMatrix = eulerAnglesToRotationMatrix(attitude);
//     Eigen::Vector3f accelWorld = rotationMatrix * accel;
//     accelWorld(2) -= GRAVITY;

//     velocity += accelWorld * dt;
//     position += velocity * dt + 0.5f * accelWorld * dt * dt;

//     state.segment<3>(0) = position;
//     state.segment<3>(3) = velocity;
//     state.segment<3>(6) = attitude;
// }


// // Jacobian 행렬 계산
// void EKF::computeJacobian(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     jacobian = Eigen::MatrixXf::Identity(9, 9);
//     jacobian.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

//     Eigen::Vector3f attitude = state.segment<3>(6);

//     Eigen::Matrix3f rotationMatrix = eulerAnglesToRotationMatrix(attitude);
//     Eigen::Matrix3f dRotation_dRoll = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitX());
//     Eigen::Matrix3f dRotation_dPitch = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitY());
//     Eigen::Matrix3f dRotation_dYaw = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitZ());

//     jacobian.block<3, 1>(3, 6) = dRotation_dRoll * accel * dt;
//     jacobian.block<3, 1>(3, 7) = dRotation_dPitch * accel * dt;
//     jacobian.block<3, 1>(3, 8) = dRotation_dYaw * accel * dt;
// }

// // Skew 대칭 행렬 계산
// Eigen::Matrix3f EKF::skewSymmetric(const Eigen::Vector3f& v) {
//     Eigen::Matrix3f skew;
//     skew <<      0, -v.z(),  v.y(),
//              v.z(),      0, -v.x(),
//             -v.y(),  v.x(),      0;
//     return skew;
// }

// // GPS 데이터를 사용한 업데이트 함수
// void EKF::update(const Eigen::Vector3f& gpsPos, const Eigen::Vector3f& gpsVel) {
//     // gpsPos는 이미 위도(degree), 경도(degree), 고도(m)로 들어옴
//     Eigen::Vector3f gpsPos_latlon = gpsPos;  // 위도, 경도, 고도 값 그대로 사용

//     // 속도를 mm/s에서 m/s로 변환
//     Eigen::Vector3f gpsVel_m = gpsVel / 1000.0f;

//     // 측정 벡터
//     Eigen::VectorXf z(6);
//     z.segment<3>(0) = gpsPos_latlon;  // 위치는 위도, 경도, 고도
//     z.segment<3>(3) = gpsVel_m;  // 속도는 m/s 단위

//     // 예측된 측정값
//     Eigen::VectorXf z_pred(6);
//     z_pred.segment<3>(0) = state.segment<3>(0);  // 예측된 위치 (위도, 경도, 고도)
//     z_pred.segment<3>(3) = state.segment<3>(3);  // 예측된 속도

//     // 측정 잔차
//     Eigen::VectorXf y = z - z_pred;

//     // 측정 행렬 H
//     Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6, 9);
//     H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
//     H.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();

//     // 칼만 이득 계산
//     Eigen::MatrixXf S = H * covariance * H.transpose() + measurementNoise;
//     Eigen::MatrixXf K = covariance * H.transpose() * S.inverse();

//     // 상태 업데이트
//     state += K * y;

//     // 공분산 업데이트
//     covariance = (Eigen::MatrixXf::Identity(9, 9) - K * H) * covariance;
// }

// // 현재 상태 반환 (위치: 위도, 경도, 고도, 각도는 도 단위)
// Eigen::VectorXf EKF::getState() const {
//     Eigen::VectorXf stateOut(9);

//     // 위치는 그대로 반환 (위도: degree, 경도: degree, 고도: m)
//     stateOut.segment<3>(0) = state.segment<3>(0);

//     // 속도
//     stateOut.segment<3>(3) = state.segment<3>(3);

//     // 자세 각도는 도 단위로 변환
//     stateOut(6) = radToDeg(state(6));  // 롤
//     stateOut(7) = radToDeg(state(7));  // 피치
//     stateOut(8) = radToDeg(state(8));  // 요

//     return stateOut;
// }

// // 오일러사용
// #include "ekf.h"
// #include <Eigen/Dense>
// #include <cmath>
// #include <iostream>

// // 상수
// const float GRAVITY = 9.80665f;  // 중력 상수
// const float GYRO_THRESHOLD = 0.001f;  // 자이로 변화 임계값 (필터링 강화)
// const float ALPHA = 0.9f;  // 저주파 필터 계수

// // 유틸리티 함수
// float radToDeg(float rad) {
//     return rad * (180.0f / M_PI);
// }

// float degToRad(float deg) {
//     return deg * (M_PI / 180.0f);
// }

// // 유효한 값인지 확인하는 함수
// bool isValidValue(float value) {
//     return !std::isnan(value) && !std::isinf(value);
// }

// // 생성자
// EKF::EKF() {
//     // 초기 상태와 공분산 설정
//     state = Eigen::VectorXf::Zero(9);
//     covariance = Eigen::MatrixXf::Identity(9, 9) * 0.05f;

//     // 프로세스 노이즈 행렬 초기화
//     processNoise = Eigen::MatrixXf::Zero(9, 9);
//     processNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.05f;  // 위치 노이즈
//     processNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.05f;  // 속도 노이즈
//     processNoise.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() * 0.001f; // 자세 노이즈

//     // 측정 노이즈 행렬 초기화
//     measurementNoise = Eigen::MatrixXf::Zero(6, 6);
//     measurementNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.01f; // 위치 측정 노이즈
//     measurementNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.01f; // 속도 측정 노이즈

//     accelLast = Eigen::Vector3f::Zero();
//     gyroLast = Eigen::Vector3f::Zero();
// }

// // 소멸자
// EKF::~EKF() {}

// // 오일러 각도를 회전 행렬로 변환하는 함수
// Eigen::Matrix3f EKF::eulerAnglesToRotationMatrix(const Eigen::Vector3f& eulerAngles) const {
//     float roll = eulerAngles(0);
//     float pitch = eulerAngles(1);
//     float yaw = eulerAngles(2);

//     Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

//     Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
//     return q.matrix();
// }

// // 저주파 필터 적용 함수
// Eigen::Vector3f EKF::lowPassFilter(const Eigen::Vector3f& input, const Eigen::Vector3f& last, float alpha) {
//     return alpha * last + (1.0f - alpha) * input;
// }

// // 예측 함수
// void EKF::predict(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     if (!isValidValue(accel.norm()) || !isValidValue(gyro.norm())) {
//         std::cerr << "유효하지 않은 IMU 데이터 감지됨" << std::endl;
//         return;
//     }

//     // 저주파 필터 적용
//     Eigen::Vector3f filteredAccel = lowPassFilter(accel, accelLast, ALPHA);
//     Eigen::Vector3f filteredGyro = lowPassFilter(gyro, gyroLast, ALPHA);
    
//     if (filteredGyro.norm() < GYRO_THRESHOLD) {
//         // 자이로 변화가 작을 때 자세를 업데이트하지 않음
//         return;
//     }

//     predictState(filteredAccel, filteredGyro, dt);
//     computeJacobian(filteredAccel, filteredGyro, dt);
//     covariance = jacobian * covariance * jacobian.transpose() + processNoise;

//     accelLast = filteredAccel;
//     gyroLast = filteredGyro;
// }

// // 상태 예측 함수
// void EKF::predictState(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     Eigen::Vector3f position = state.segment<3>(0);
//     Eigen::Vector3f velocity = state.segment<3>(3);
//     Eigen::Vector3f attitude = state.segment<3>(6);

//     // 자세 업데이트
//     attitude += gyro * dt;

//     // 각도 정상화
//     for (int i = 0; i < 3; ++i) {
//         if (attitude(i) > M_PI)
//             attitude(i) -= 2 * M_PI;
//         else if (attitude(i) < -M_PI)
//             attitude(i) += 2 * M_PI;
//     }

//     // 회전 행렬 계산
//     Eigen::Matrix3f rotationMatrix = eulerAnglesToRotationMatrix(attitude);
//     Eigen::Vector3f accelWorld = rotationMatrix * accel;
//     accelWorld(2) -= GRAVITY;

//     // 속도와 위치 업데이트
//     velocity += accelWorld * dt;
//     position += velocity * dt + 0.5f * accelWorld * dt * dt;

//     state.segment<3>(0) = position;
//     state.segment<3>(3) = velocity;
//     state.segment<3>(6) = attitude;
// }

// // Jacobian 행렬 계산 함수
// void EKF::computeJacobian(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
//     jacobian = Eigen::MatrixXf::Identity(9, 9);
//     jacobian.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

//     Eigen::Vector3f attitude = state.segment<3>(6);

//     Eigen::Matrix3f rotationMatrix = eulerAnglesToRotationMatrix(attitude);
//     Eigen::Matrix3f dRotation_dRoll = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitX());
//     Eigen::Matrix3f dRotation_dPitch = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitY());
//     Eigen::Matrix3f dRotation_dYaw = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitZ());

//     jacobian.block<3, 1>(3, 6) = dRotation_dRoll * accel * dt;
//     jacobian.block<3, 1>(3, 7) = dRotation_dPitch * accel * dt;
//     jacobian.block<3, 1>(3, 8) = dRotation_dYaw * accel * dt;
// }

// // 스큐 대칭 행렬 계산 함수
// Eigen::Matrix3f EKF::skewSymmetric(const Eigen::Vector3f& v) {
//     Eigen::Matrix3f skew;
//     skew <<      0, -v.z(),  v.y(),
//              v.z(),      0, -v.x(),
//             -v.y(),  v.x(),      0;
//     return skew;
// }

// // GPS 데이터를 사용한 업데이트 함수
// void EKF::update(const Eigen::Vector3f& gpsPos, const Eigen::Vector3f& gpsVel) {
//     Eigen::Vector3f gpsPos_latlon = gpsPos;

//     Eigen::Vector3f gpsVel_m = gpsVel / 1000.0f;  // mm/s에서 m/s로 변환

//     Eigen::VectorXf z(6);
//     z.segment<3>(0) = gpsPos_latlon;
//     z.segment<3>(3) = gpsVel_m;

//     Eigen::VectorXf z_pred(6);
//     z_pred.segment<3>(0) = state.segment<3>(0);
//     z_pred.segment<3>(3) = state.segment<3>(3);

//     // 측정 잔차 계산
//     Eigen::VectorXf y = z - z_pred;

//     // 측정 행렬 H 정의
//     Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6, 9);
//     H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
//     H.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();

//     // 칼만 이득 계산
//     Eigen::MatrixXf S = H * covariance * H.transpose() + measurementNoise;
//     Eigen::MatrixXf K = covariance * H.transpose() * S.inverse();

//     // 상태 업데이트
//     state += K * y;

//     // 공분산 업데이트
//     covariance = (Eigen::MatrixXf::Identity(9, 9) - K * H) * covariance;
// }

// // 현재 상태 반환 함수 (위치: 위도, 경도, 고도, 각도는 도 단위)
// Eigen::VectorXf EKF::getState() const {
//     Eigen::VectorXf stateOut(9);

//     // 위치는 그대로 반환 (위도: degree, 경도: degree, 고도: m)
//     stateOut.segment<3>(0) = state.segment<3>(0);

//     // 속도
//     stateOut.segment<3>(3) = state.segment<3>(3);

//     // 자세 각도는 도 단위로 변환
//     stateOut(6) = radToDeg(state(6));  // 롤
//     stateOut(7) = radToDeg(state(7));  // 피치
//     stateOut(8) = radToDeg(state(8));  // 요

//     return stateOut;
// }

// 쿼터니언 사용
#include "ekf.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// 상수
const float GRAVITY = 9.80665f;       // 중력 상수
const float GYRO_THRESHOLD = 0.01f;   // 자이로 변화 임계값 강화
const float ALPHA = 0.98f;            // 저주파 필터 계수 증가

// 유틸리티 함수
float radToDeg(float rad) {
    return rad * (180.0f / M_PI);
}

float degToRad(float deg) {
    return deg * (M_PI / 180.0f);
}

bool isValidValue(float value) {
    return !std::isnan(value) && !std::isinf(value);
}

// 생성자
EKF::EKF() {
    state = Eigen::VectorXf::Zero(10);
    state(6) = 1.0f;  // 초기 쿼터니언 단위값 설정 (w=1)
    covariance = Eigen::MatrixXf::Identity(10, 10) * 0.05f;

    processNoise = Eigen::MatrixXf::Zero(10, 10);
    processNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.05f;
    processNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.05f;
    processNoise.block<4, 4>(6, 6) = Eigen::Matrix4f::Identity() * 0.001f;

    measurementNoise = Eigen::MatrixXf::Zero(6, 6);
    measurementNoise.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * 0.05f;
    measurementNoise.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * 0.05f;

    accelLast = Eigen::Vector3f::Zero();
    gyroLast = Eigen::Vector3f::Zero();
}

EKF::~EKF() {}

Eigen::Matrix3f EKF::quaternionToRotationMatrix(const Eigen::Quaternionf& q) const {
    return q.toRotationMatrix();
}

Eigen::Vector3f EKF::lowPassFilter(const Eigen::Vector3f& input, const Eigen::Vector3f& last, float alpha) {
    return alpha * last + (1.0f - alpha) * input;
}

// 예측 함수
void EKF::predict(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
    if (!isValidValue(accel.norm()) || !isValidValue(gyro.norm())) {
        std::cerr << "유효하지 않은 IMU 데이터 감지됨" << std::endl;
        return;
    }

    Eigen::Vector3f filteredAccel = lowPassFilter(accel, accelLast, ALPHA);
    Eigen::Vector3f filteredGyro = lowPassFilter(gyro, gyroLast, ALPHA);
    
    if (filteredGyro.norm() < GYRO_THRESHOLD) {
        return; // 작은 자이로 값에서는 업데이트 생략
    }

    predictState(filteredAccel, filteredGyro, dt);
    computeJacobian(filteredAccel, filteredGyro, dt);
    covariance = jacobian * covariance * jacobian.transpose() + processNoise;

    accelLast = filteredAccel;
    gyroLast = filteredGyro;
}

void EKF::predictState(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
    if (gyro.norm() < GYRO_THRESHOLD) {
        return;  // 작은 자이로 값에서는 업데이트 생략
    }
    
    Eigen::Vector3f position = state.segment<3>(0);
    Eigen::Vector3f velocity = state.segment<3>(3);
    Eigen::Quaternionf attitude(state(6), state(7), state(8), state(9));

    Eigen::Quaternionf deltaQ;
    deltaQ.w() = 1.0f;
    deltaQ.vec() = (gyro * 0.5f * dt).normalized();
    attitude = (attitude * deltaQ).normalized();

    attitude.normalize(); // 드리프트 방지용 정규화

    Eigen::Matrix3f rotationMatrix = quaternionToRotationMatrix(attitude);
    Eigen::Vector3f accelWorld = rotationMatrix * accel;
    accelWorld(2) -= GRAVITY;

    velocity += accelWorld * dt;
    position += velocity * dt + 0.5f * accelWorld * dt * dt;

    // 작은 상태 변동 제한 (오차 누적 방지)
    if (velocity.norm() > 1e-6 || position.norm() > 1e-6) {
        state.segment<3>(0) = position;
        state.segment<3>(3) = velocity;
    }

    state(6) = attitude.w();
    state.segment<3>(7) = attitude.vec();
}

void EKF::computeJacobian(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt) {
    jacobian = Eigen::MatrixXf::Identity(10, 10);
    jacobian.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;

    Eigen::Quaternionf attitude(state(6), state(7), state(8), state(9));

    Eigen::Matrix3f rotationMatrix = quaternionToRotationMatrix(attitude);
    Eigen::Matrix3f dRotation_dRoll = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitX());
    Eigen::Matrix3f dRotation_dPitch = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitY());
    Eigen::Matrix3f dRotation_dYaw = skewSymmetric(rotationMatrix * Eigen::Vector3f::UnitZ());

    jacobian.block<3, 1>(3, 6) = dRotation_dRoll * accel * dt;
    jacobian.block<3, 1>(3, 7) = dRotation_dPitch * accel * dt;
    jacobian.block<3, 1>(3, 8) = dRotation_dYaw * accel * dt;
}

Eigen::Matrix3f EKF::skewSymmetric(const Eigen::Vector3f& v) {
    Eigen::Matrix3f skew;
    skew <<      0, -v.z(),  v.y(),
             v.z(),      0, -v.x(),
            -v.y(),  v.x(),      0;
    return skew;
}

void EKF::update(const Eigen::Vector3f& gpsPos, const Eigen::Vector3f& gpsVel) {
    Eigen::Vector3f gpsPos_latlon = gpsPos;
    Eigen::Vector3f gpsVel_m = gpsVel / 1000.0f;

    Eigen::VectorXf z(6);
    z.segment<3>(0) = gpsPos_latlon;
    z.segment<3>(3) = gpsVel_m;

    Eigen::VectorXf z_pred(6);
    z_pred.segment<3>(0) = state.segment<3>(0);
    z_pred.segment<3>(3) = state.segment<3>(3);

    Eigen::VectorXf y = z - z_pred;

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6, 10);
    H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
    H.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();

    Eigen::MatrixXf S = H * covariance * H.transpose() + measurementNoise;
    Eigen::MatrixXf K = covariance * H.transpose() * S.inverse();

    state += K * y;
    covariance = (Eigen::MatrixXf::Identity(10, 10) - K * H) * covariance;
}

Eigen::VectorXf EKF::getState() const {
    Eigen::VectorXf stateOut(10);

    stateOut.segment<3>(0) = state.segment<3>(0);
    stateOut.segment<3>(3) = state.segment<3>(3);

    stateOut(6) = state(6);
    stateOut.segment<3>(7) = state.segment<3>(7);

    return stateOut;
}