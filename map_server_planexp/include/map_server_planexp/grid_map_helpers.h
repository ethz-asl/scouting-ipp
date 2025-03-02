//
// Created by Friedrich M. Rockenbauer on 01.11.24.
//

#ifndef MAP_SERVER_PLANEXP_GRID_MAP_HELPERS_H
#define MAP_SERVER_PLANEXP_GRID_MAP_HELPERS_H

template<typename EigenType_, typename MultiArrayMessageType_>
bool matrixEigenCopyToMultiArrayMessage(const EigenType_& e, MultiArrayMessageType_& m)
{
  m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.size();
  m.layout.dim[0].size = e.outerSize();
  m.layout.dim[1].stride = e.innerSize();
  m.layout.dim[1].size = e.innerSize();

  if (e.IsRowMajor) {
    m.layout.dim[0].label = "row_index";
    m.layout.dim[1].label = "column_index";
  } else {
    m.layout.dim[0].label = "column_index";
    m.layout.dim[1].label = "row_index";
  }

  m.data.insert(m.data.begin() + m.layout.data_offset, e.data(), e.data() + e.size());
  return true;
}


#endif //MAP_SERVER_PLANEXP_GRID_MAP_HELPERS_H
