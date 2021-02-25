#include "Utils.h"


namespace clothsim {

void AddToSparseMatrix(SparseMatrixXd& matrix,
                       const Eigen::Matrix3d& data,
                       size_t row,
                       size_t col) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      matrix.coeffRef(row + i, col + j) += data(i, j);
    }
  }
}

void SubToSparseMatrix(SparseMatrixXd& matrix,
                       const Eigen::Matrix3d& data,
                       size_t row,
                       size_t col) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      matrix.coeffRef(row + i, col + j) -= data(i, j);
    }
  }
}

}  // namespace clothsim