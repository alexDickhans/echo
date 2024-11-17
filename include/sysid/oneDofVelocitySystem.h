#pragma once

#include "utils.h"

class OneDofVelocitySystem {
public:
    Eigen::RowVector3d ff;

public:
    explicit OneDofVelocitySystem(const Eigen::Vector3d &ff) : ff(ff) {
    }

    OneDofVelocitySystem() = default;

    void characterize(const std::vector<double>& x, const std::vector<double>& &u) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(x.size() - 1, 3);
        Eigen::MatrixXd b = Eigen::VectorXd::Zero(x.size() - 1);

        for (int i = 0; i < x.size() - 1; i++) {
            A(i, 0) = x[i];
            A(i, 1) = u[i];
            A(i, 2) = -signnum(x[i]);
            b(i) = x[i + 1];
        }

        Eigen::VectorXd solution = A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

        double K_s = -solution(2) / solution(1);
        double K_v = (1 - solution(0)) / solution(1);
        double K_a = -K_v * 0.01 / log(solution(0));

        ff = {K_v, K_a, K_s};
    }

    void characterize(const std::function<double()> &x, const std::function<double()> &u,
                      const std::function<bool()> &stop) {
        std::vector<double> xRecorded;
        std::vector<double> uRecorded;

        while (!stop()) {
            // Arcade control scheme
            double u_k = u(); // Gets the turn left/right from right joystick
            double x_k = x();

            uRecorded.emplace_back(u_k);
            xRecorded.emplace_back(x_k);

            pros::delay(10); // Run for 20 ms then update
        }

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(xRecorded.size() - 1, 3);
        Eigen::MatrixXd b = Eigen::VectorXd::Zero(xRecorded.size() - 1);

        for (int i = 0; i < xRecorded.size() - 1; i++) {
            A(i, 0) = xRecorded[i];
            A(i, 1) = uRecorded[i];
            A(i, 2) = -signnum(xRecorded[i]);
            b(i) = xRecorded[i + 1];
        }

        Eigen::VectorXd solution = A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

        double K_s = -solution(2) / solution(1);
        double K_v = (1 - solution(0)) / solution(1);
        double K_a = -K_v * 0.01 / log(solution(0));

        ff = {K_v, K_a, K_s};
    }

    double evaluate(Eigen::Vector3d x) const {
        return ff * x;
    }

    Eigen::RowVector3d getFF() {
        return ff;
    }
};
