//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAV_SIMULATOR_CONVERSIONS_H
#define MAV_SIMULATOR_CONVERSIONS_H

#include <Eigen/Eigen>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

template <class Derived>
void matrixEigenToIntMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Int32MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}
template <class Derived>
void matrixEigenToFloatMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}


#endif //MAV_SIMULATOR_CONVERSIONS_H
