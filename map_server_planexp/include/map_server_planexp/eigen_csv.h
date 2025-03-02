//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAP_SERVER_PLANEXP_EIGEN_CSV_H
#define MAP_SERVER_PLANEXP_EIGEN_CSV_H

#include <Eigen/Eigen>
#include <fstream>

// define the format you want, you only need one instance of this...
// see https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
// writing functions taking Eigen types as parameters,
// see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
template <typename Derived>
void writeToCSVfile(std::string name, const Eigen::MatrixBase<Derived>& matrix)
{
  std::ofstream file(name.c_str());
  file << matrix.format(CSVFormat);
  // file.close() is not necessary,
  // desctructur closes file, see https://en.cppreference.com/w/cpp/io/basic_ofstream
}
template<typename M>
M load_csv (const std::string & path) {
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<typename M::Scalar> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      values.push_back(std::stod(cell));
    }
    ++rows;
  }
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

#endif //MAP_SERVER_PLANEXP_EIGEN_CSV_H
