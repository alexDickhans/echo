#pragma once

#include "utils.h"

class OneDofVelocitySystem {
public:
    Eigen::RowVector3d ff;

public:
    explicit OneDofVelocitySystem(const Eigen::Vector3d &ff) : ff(ff) {
    }

    OneDofVelocitySystem() = default;

    void characterize(const std::vector<double>& x, const std::vector<double>& u) {

        size_t offset = 1;

        // Allocate large enough matrices for linear regression
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(x.size() - offset, 3);
        Eigen::VectorXd c = Eigen::VectorXd::Zero(x.size() - offset);

        // Fill the matrix with recorded inputs and states
        for (int i = offset; i < x.size(); i++) {
            A(i-offset, 0) = x[i];
            A(i-offset, 1) = (x[i] - x[i-1]) * 100.0;
            A(i-offset, 2) = signnum_c(x[i]);
            if (i-1 < x.size()- offset) {
                c(i-1) = u[i - offset];
            }
        }

        // Compute the least-squares solution with SVD, which provides the most stability and accuracy of the solutions
        Eigen::VectorXd b = A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(c);

        // Set the feedforward to the linear system solution
        ff = b.transpose();
    }

    void characterize(const std::function<double()> &x, const std::function<double()> &u,
                      const std::function<bool()> &stop) {
        std::vector<double> xRecorded;
        std::vector<double> uRecorded;

        while (!stop()) {
            double u_k = u();
            double x_k = x();

            uRecorded.emplace_back(u_k);
            xRecorded.emplace_back(x_k);

            pros::delay(10);
        }

        characterize(xRecorded, uRecorded);
    }

    double evaluate(Eigen::Vector3d x) const {
        return ff * x;
    }

    Eigen::RowVector3d getFF() {
        return ff;
    }
};
