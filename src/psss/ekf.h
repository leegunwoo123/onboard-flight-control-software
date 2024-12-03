// 쿼터니언 사용
#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>

class EKF {
public:
    EKF();
    ~EKF();

    void predict(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt);
    void updateWithGPS(const Eigen::Vector3f& gpsPos, const Eigen::Vector3f& gpsVel);
    void updateWithMag(const Eigen::Vector3f& mag);  // 자기장 업데이트 함수
    Eigen::VectorXf getState() const;
private:
    Eigen::VectorXf state;  // 상태 벡터 (위치 3, 속도 3, 자세 4)
    Eigen::MatrixXf covariance;  // 오차 공분산 행렬
    Eigen::MatrixXf processNoise;  // 프로세스 노이즈 행렬
    Eigen::MatrixXf measurementNoise;  // 측정 노이즈 행렬
    Eigen::MatrixXf jacobian;  // Jacobian 행렬

    Eigen::Vector3f accelLast;  // 마지막 가속도 값
    Eigen::Vector3f gyroLast;   // 마지막 자이로 값 (저주파 필터에 사용)

    Eigen::Matrix3f quaternionToRotationMatrix(const Eigen::Quaternionf& q) const;
    void computeJacobian(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt);  
    void predictState(const Eigen::Vector3f& accel, const Eigen::Vector3f& gyro, float dt);
    Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);

    Eigen::Vector3f lowPassFilter(const Eigen::Vector3f& input, const Eigen::Vector3f& last, float alpha);
};

#endif