//
// File: unique.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "unique.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double b_data[]
//                int nb
//                int &nFinite
//                int &nPInf
//                int &nNaN
// Return Type  : int
//
namespace coder {
int count_nonfinites(const double b_data[], int nb, int &nFinite, int &nPInf,
                     int &nNaN)
{
  int k;
  int nMInf;
  boolean_T exitg1;
  k = 0;
  while ((k + 1 <= nb) && std::isinf(b_data[k]) && (b_data[k] < 0.0)) {
    k++;
  }
  nMInf = k;
  k = nb;
  while ((k >= 1) && std::isnan(b_data[k - 1])) {
    k--;
  }
  nNaN = nb - k;
  exitg1 = false;
  while ((!exitg1) && (k >= 1)) {
    double d;
    d = b_data[k - 1];
    if (std::isinf(d) && (d > 0.0)) {
      k--;
    } else {
      exitg1 = true;
    }
  }
  nPInf = (nb - k) - nNaN;
  nFinite = k - nMInf;
  return nMInf;
}

//
// Arguments    : const double a[1000]
//                double b_data[]
//                int b_size[2]
//                int ndx_data[]
//                int pos[1000]
// Return Type  : int
//
int unique_vector(const double a[1000], double b_data[], int b_size[2],
                  int ndx_data[], int pos[1000])
{
  double x;
  int idx[1000];
  int iwork[1000];
  int b_i;
  int i;
  int i2;
  int j;
  int k0;
  int ndx_size;
  int p;
  int pEnd;
  int qEnd;
  for (k0 = 0; k0 <= 998; k0 += 2) {
    x = a[k0 + 1];
    if ((a[k0] <= x) || std::isnan(x)) {
      idx[k0] = k0 + 1;
      idx[k0 + 1] = k0 + 2;
    } else {
      idx[k0] = k0 + 2;
      idx[k0 + 1] = k0 + 1;
    }
  }
  i = 2;
  while (i < 1000) {
    i2 = i << 1;
    j = 1;
    for (pEnd = i + 1; pEnd < 1001; pEnd = qEnd + i) {
      int kEnd;
      int q;
      p = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > 1001) {
        qEnd = 1001;
      }
      k0 = 0;
      kEnd = qEnd - j;
      while (k0 + 1 <= kEnd) {
        x = a[idx[q] - 1];
        b_i = idx[p - 1];
        if ((a[b_i - 1] <= x) || std::isnan(x)) {
          iwork[k0] = b_i;
          p++;
          if (p == pEnd) {
            while (q + 1 < qEnd) {
              k0++;
              iwork[k0] = idx[q];
              q++;
            }
          }
        } else {
          iwork[k0] = idx[q];
          q++;
          if (q + 1 == qEnd) {
            while (p < pEnd) {
              k0++;
              iwork[k0] = idx[p - 1];
              p++;
            }
          }
        }
        k0++;
      }
      for (k0 = 0; k0 < kEnd; k0++) {
        idx[(j + k0) - 1] = iwork[k0];
      }
      j = qEnd;
    }
    i = i2;
  }
  b_size[0] = 1;
  for (k0 = 0; k0 < 1000; k0++) {
    b_data[k0] = a[idx[k0] - 1];
  }
  pEnd = count_nonfinites(b_data, 1000, k0, i2, p);
  ndx_size = 0;
  if (pEnd > 0) {
    ndx_size = 1;
    b_i = static_cast<unsigned short>(pEnd);
    for (j = 0; j < b_i; j++) {
      pos[idx[j] - 1] = 1;
    }
  }
  i = pEnd + k0;
  while (pEnd + 1 <= i) {
    x = b_data[pEnd];
    k0 = pEnd;
    do {
      pEnd++;
    } while (!((pEnd + 1 > i) || (b_data[pEnd] != x)));
    ndx_size++;
    b_data[ndx_size - 1] = x;
    for (j = k0 + 1; j <= pEnd; j++) {
      pos[idx[j - 1] - 1] = ndx_size;
    }
    idx[ndx_size - 1] = idx[k0];
  }
  if (i2 > 0) {
    ndx_size++;
    b_data[ndx_size - 1] = b_data[i];
    b_i = static_cast<unsigned short>(i2);
    for (j = 0; j < b_i; j++) {
      pos[idx[i + j] - 1] = ndx_size;
    }
    idx[ndx_size - 1] = idx[i];
  }
  pEnd = (i + i2) - 1;
  b_i = static_cast<unsigned short>(p);
  for (j = 0; j < b_i; j++) {
    i = ndx_size + j;
    k0 = (pEnd + j) + 1;
    b_data[i] = b_data[k0];
    k0 = idx[k0];
    pos[k0 - 1] = i + 1;
    idx[i] = k0;
  }
  if (static_cast<unsigned short>(p) - 1 >= 0) {
    ndx_size += static_cast<unsigned short>(p);
  }
  if (ndx_size < 1) {
    b_size[1] = 0;
  } else {
    b_size[1] = ndx_size;
  }
  b_i = static_cast<unsigned short>(ndx_size);
  if (b_i - 1 >= 0) {
    std::copy(&idx[0], &idx[b_i], &ndx_data[0]);
  }
  return ndx_size;
}

} // namespace coder

//
// File trailer for unique.cpp
//
// [EOF]
//
