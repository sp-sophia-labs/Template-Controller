#include <iostream>
#include <eigen3/Eigen/Dense>
#include <array>


inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

int main()
{
    std::array<double, 42> jacobian_array = {
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
        7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
        7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
        7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0
    };
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_transpose_pinv;

    try
    {
      // Eigen::MatrixXd inverseMatrix = jacobian.inverse();
      Eigen::MatrixXd pseudoinverseMatrix = 
      Eigen::JacobiSVD<Eigen::MatrixXd>
      (jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Eigen::MatrixXd::Identity(
        jacobian.rows(), jacobian.cols()));
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }

    return 0;
}

