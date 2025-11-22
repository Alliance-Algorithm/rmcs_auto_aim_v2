#include "tf.hpp"
#include <eigen3/Eigen/Dense>

namespace rmcs::util {

auto operator*(const JointTransform& tf1, const JointTransform& tf2) -> JointTransform {
    auto t1 = tf1.translation.make<Eigen::Vector3d>();
    auto t2 = tf2.translation.make<Eigen::Vector3d>();

    auto q1 = tf1.orientation.make<Eigen::Quaterniond>();
    auto q2 = tf2.orientation.make<Eigen::Quaterniond>();

    return JointTransform { Translation { q1 * t2 + t1 }, Orientation { q1 * q2 } };
}

auto JointTransform::inverse() const noexcept -> JointTransform {
    auto t = translation.make<Eigen::Vector3d>();
    auto q = orientation.make<Eigen::Quaterniond>();

    auto q_inv = q.inverse();
    auto t_inv = -(q_inv * t);

    return JointTransform { Translation { t_inv }, Orientation { q_inv } };
}

}
