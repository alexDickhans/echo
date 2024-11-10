#pragma once

#include "Eigen/Dense"
#include "feedback/feedback.h"
#include "utils/linear.h"

class LQR {
private:
    Eigen::VectorXf ref;
    Eigen::MatrixXf A, B, Q, R;
    Eigen::MatrixXf K;

    QTime dt;

public:
    LQR(const Eigen::VectorXf &ref, const Eigen::MatrixXf &a, const Eigen::MatrixXf &b, const Eigen::MatrixXf &q,
        const Eigen::MatrixXf &r, const QTime &dt) : ref(ref), A(a), B(b), Q(q), R(r), dt(dt) {

        auto discAB = discretizeAB(A, B, dt);

        auto S = dareSolver(discAB.first, discAB.second, Q, R);

        K = (R + discAB.second.transpose() * S * discAB.second).inverse() *
                                discAB.second.transpose() * S * discAB.first;
    }

    Eigen::VectorXf update(const Eigen::VectorXf& x) const {
        return K * (ref-x);
    }

    ~LQR() = default;
};
