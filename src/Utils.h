#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace clothsim {

typedef Eigen::Matrix<double, -1, 3, Eigen::RowMajor> MatrixRX3d;
typedef Eigen::SparseMatrix<double> SparseMatrixXd;

void AddToSparseMatrix(SparseMatrixXd& matrix,
                       const Eigen::Matrix3d& data,
                       size_t row,
                       size_t col);

void SubToSparseMatrix(SparseMatrixXd& matrix,
                       const Eigen::Matrix3d& data,
                       size_t row,
                       size_t col);
}