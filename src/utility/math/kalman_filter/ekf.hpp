#pragma once
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Cholesky>

namespace rmcs::util {

inline auto DefaultAdd = [](auto const& a, auto const& b) { return a + b; };

inline auto DefaultSubtract = [](auto const& a, auto const& b) { return a - b; };

// 约束状态转移函数: f(x) -> x_next
template <typename Modelfunc, int StateDim>
concept StateTransitionModel = requires(Modelfunc f, Eigen::Matrix<double, StateDim, 1> x) {
    { f(x) } -> std::convertible_to<Eigen::Matrix<double, StateDim, 1>>;
};

// 约束观测函数: h(x) -> z
template <typename H, int StateDim, int ObsDim>
concept ObservationModel = requires(H h, Eigen::Matrix<double, StateDim, 1> x) {
    { h(x) } -> std::convertible_to<Eigen::Matrix<double, ObsDim, 1>>;
};

// 约束雅可比计算: Jacobian(x) -> Matrix
template <typename JacobianFunc, int Rows, int Cols>
concept JacobianModel = requires(JacobianFunc j, Eigen::Matrix<double, Cols, 1> x) {
    { j(x) } -> std::convertible_to<Eigen::Matrix<double, Rows, Cols>>;
};

template <int StateDim, int ObsDim>
class EKF {
public:
    using XVec = Eigen::Matrix<double, StateDim, 1>;
    using ZVec = Eigen::Matrix<double, ObsDim, 1>;
    using AMat = Eigen::Matrix<double, StateDim, StateDim>;
    using PMat = Eigen::Matrix<double, StateDim, StateDim>;
    using PDig = Eigen::Matrix<double, StateDim, 1>;
    using RMat = Eigen::Matrix<double, ObsDim, ObsDim>;
    using RDig = Eigen::Matrix<double, ObsDim, 1>;
    using QMat = Eigen::Matrix<double, StateDim, StateDim>;
    using HMat = Eigen::Matrix<double, ObsDim, StateDim>;

    XVec x;

    explicit EKF()
        : x { XVec::Zero() }
        , P_ { PMat::Zero() } { }

    explicit EKF(XVec const& initial_x, PMat const& initial_P)
        : x(initial_x)
        , P_(initial_P) { }

    /**
     * @brief 预测步
     * @param f 状态转移 Lambda: (XVec) -> XVec
     * @param get_F 雅可比 Lambda: (XVec) -> XMat
     * @param Q 过程噪声
     * @param x_add_op 状态更新算子 (默认线性加)
     * 公式:
     * 1. x_pre = f(x)
     * 2. P_pre = F * P * F^T + Q
     */
    template <typename ModelFunc, typename JacobianFunc>
    auto predict(ModelFunc&& f, JacobianFunc&& get_F, QMat const& Q) -> void {
        static_assert(StateTransitionModel<ModelFunc, StateDim>,
            "\n[EKF Error] 状态转移函数 f(x) 签名错误！\n"
            "预期格式: XVec f(const XVec&)\n");

        static_assert(JacobianModel<JacobianFunc, StateDim, StateDim>,
            "\n[EKF Error] 状态雅可比函数 get_F(x) 签名错误！\n"
            "预期格式: AMat get_F(const XVec&)\n"
            "请检查返回矩阵是否为 [StateDim x StateDim] 维度.");

        // x_{k|k-1} = f(x_{k-1|k-1}, u_k)
        auto x_pre = f(x);

        // F = df/dx | x=x_{k-1}
        auto F = get_F(x);

        // P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
        auto P_pre = F * P_ * F.transpose() + Q;

        // 更新内部状态
        x  = x_pre;
        P_ = P_pre;
    }

    /**
     * @brief 更新步
     * @param z 观测值
     * @param h 观测 Lambda: (XVec) -> ZVec
     * @param get_H 雅可比 Lambda: (XVec) -> ZMat
     * @param R 观测噪声
     * @param x_add_op 状态更新算子 ⊕
     * @param z_sub_op 残差计算算子 ⊖
     * 公式:
     * 1. y = z - h(x_pre)           (残差/创新)
     * 2. S = H * P_pre * H^T + R    (创新协方差)
     * 3. K = P_pre * H^T * S^-1     (卡尔曼增益)
     * 4. x = x_pre + K * y          (状态更新)
     * 5. P = (I - K * H) * P_pre    (协方差更新)
     * or Joseph Form:
     * P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T (协方差更新)
     */
    template <typename MeasFunc, typename JacobianFunc, typename AddOp = decltype(DefaultAdd),
        typename SubOp = decltype(DefaultSubtract)>
    auto update(ZVec const& z, MeasFunc&& h, JacobianFunc&& get_H, RMat const& R, AddOp&& x_add_op,
        SubOp&& z_sub_op) -> void {
        static_assert(ObservationModel<MeasFunc, StateDim, ObsDim>,
            "\n[EKF Error] 观测函数 h(x) 不符合要求！\n"
            "预期签名: Eigen::Matrix<double, ObsDim, 1> h(const Eigen::Matrix<double, "
            "StateDim,1>&)\n");

        static_assert(JacobianModel<JacobianFunc, ObsDim, StateDim>,
            "\n[EKF Error] 观测雅可比函数 get_H(x) 不符合要求！\n"
            "预期签名: Eigen::Matrix<double, ObsDim, StateDim> get_H(const "
            "Eigen::Matrix<double,StateDim, 1>&)\n");

        static_assert(std::is_invocable_r_v<XVec, AddOp, XVec, XVec>,
            "x_add_op 必须接受两个 XVec 并返回 XVec");

        // H = dh/dx | x=x_pre
        auto H = get_H(x);
        // --- 1. 计算残差 (Innovation) ---
        // y_k = z_k - h(x_{k|k-1})
        auto y = z_sub_op(z, h(x));

        // --- 2. 计算残差协方差 (Innovation Covariance) ---
        // S_k = H_k * P_{k|k-1} * H_k^T + R_k
        auto S = H * P_ * H.transpose() + R;

        // --- 3. 计算最优卡尔曼增益 (Optimal Kalman Gain) ---
        // K_k = P_{k|k-1} * H_k^T * S_k^-1
        // 使用高效的 LDLT 分解求解线性方程组 S*K^T = H*P
        auto K = P_ * H.transpose() * S.ldlt().solve(RMat::Identity());

        // --- 4. 状态后验更新 (State Update) ---
        // x_{k|k} = x_{k|k-1} + K_k * y_k
        x = x_add_op(x, K * y);

        // --- 5. 协方差后验更新 (Covariance Update) ---
        // P_{k|k} = (I - K_k * H_k) * P_{k|k-1}
        // or Joseph Form:P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T
        PMat P_next;
        const AMat I_KH  = AMat::Identity() - K * H;
        P_next.noalias() = I_KH * P_ * I_KH.transpose();
        P_next += (K * R * K.transpose()).eval();
        // 强制对称化
        P_ = 0.5 * (P_next + P_next.transpose());
    }

private:
    PMat P_;
};

}
