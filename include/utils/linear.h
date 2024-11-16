#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"
#include <utility>

std::pair<Eigen::MatrixXf, Eigen::MatrixXf> discretizeAB(
const Eigen::MatrixXf& contA, const Eigen::MatrixXf& contB, QTime dtSeconds) {

    int states = contA.rows();
    int inputs = contB.cols();

    // Create an augmented matrix M
    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();

    // Top-left block is A
    M.topLeftCorner(states, states) = contA;

    // Top-right block is B
    M.topRightCorner(states, inputs) = contB;

    // Matrix exponential of M * dt
    Eigen::MatrixXf phi = (M * dtSeconds.Convert(second)).exp();

    // Discretized A (A_d) is the top-left block of phi
    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);

    // Discretized B (B_d) is the top-right block of phi
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);

    return {discA, discB};
}

Eigen::MatrixXf dareSolver(const Eigen::MatrixXf &A, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q, const Eigen::MatrixXf &R) {
    Eigen::MatrixXf X = Q;
    Eigen::MatrixXf X_prev;
    Eigen::MatrixXf K, temp;

    for (int i = 0; i < 1000; ++i) {
        // Iterate until convergence
        X_prev = X;
        K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
        X = A.transpose() * X * (A - B * K) + Q;

        if ((X - X_prev).norm() < 1e-6) {
            break;
        }
    }

    return X;
}
