//
// File: computeGrad_StoreHx.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeGrad_StoreHx.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : g_struct_T &obj
//                const double H_data[]
//                const double f_data[]
//                const double x_data[]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void computeGrad_StoreHx(g_struct_T &obj, const double H_data[],
                         const double f_data[], const double x_data[])
{
  switch (obj.objtype) {
  case 5: {
    int i;
    i = obj.nvar;
    if (i - 2 >= 0) {
      std::memset(&obj.grad.data[0], 0,
                  static_cast<unsigned int>(i - 1) * sizeof(double));
    }
    obj.grad.data[obj.nvar - 1] = obj.gammaScalar;
  } break;
  case 3: {
    int i;
    internal::blas::c_xgemv(obj.nvar, obj.nvar, H_data, obj.nvar, x_data,
                            obj.Hx.data);
    i = static_cast<unsigned char>(obj.nvar);
    if (i - 1 >= 0) {
      std::copy(&obj.Hx.data[0], &obj.Hx.data[i], &obj.grad.data[0]);
    }
    if (obj.hasLinear && (obj.nvar >= 1)) {
      int ixlast;
      int scalarLB;
      int vectorUB;
      ixlast = obj.nvar - 1;
      scalarLB = ((ixlast + 1) / 2) << 1;
      vectorUB = scalarLB - 2;
      for (int k{0}; k <= vectorUB; k += 2) {
        __m128d r;
        r = _mm_loadu_pd(&obj.grad.data[k]);
        _mm_storeu_pd(&obj.grad.data[k],
                      _mm_add_pd(r, _mm_loadu_pd(&f_data[k])));
      }
      for (int k{scalarLB}; k <= ixlast; k++) {
        obj.grad.data[k] += f_data[k];
      }
    }
  } break;
  default: {
    __m128d r;
    int i;
    int ixlast;
    int iy;
    int scalarLB;
    int vectorUB;
    ixlast = obj.maxVar - 1;
    internal::blas::c_xgemv(obj.nvar, obj.nvar, H_data, obj.nvar, x_data,
                            obj.Hx.data);
    i = obj.nvar + 1;
    scalarLB = ((((ixlast - i) + 1) / 2) << 1) + i;
    vectorUB = scalarLB - 2;
    for (iy = i; iy <= vectorUB; iy += 2) {
      _mm_storeu_pd(
          &obj.Hx.data[iy - 1],
          _mm_mul_pd(_mm_set1_pd(obj.beta), _mm_loadu_pd(&x_data[iy - 1])));
    }
    for (iy = scalarLB; iy <= ixlast; iy++) {
      obj.Hx.data[iy - 1] = obj.beta * x_data[iy - 1];
    }
    i = static_cast<unsigned char>(ixlast);
    if (i - 1 >= 0) {
      std::copy(&obj.Hx.data[0], &obj.Hx.data[i], &obj.grad.data[0]);
    }
    if (obj.hasLinear && (obj.nvar >= 1)) {
      ixlast = obj.nvar - 1;
      scalarLB = ((ixlast + 1) / 2) << 1;
      vectorUB = scalarLB - 2;
      for (int k{0}; k <= vectorUB; k += 2) {
        r = _mm_loadu_pd(&obj.grad.data[k]);
        _mm_storeu_pd(&obj.grad.data[k],
                      _mm_add_pd(r, _mm_loadu_pd(&f_data[k])));
      }
      for (int k{scalarLB}; k <= ixlast; k++) {
        obj.grad.data[k] += f_data[k];
      }
    }
    ixlast = (obj.maxVar - obj.nvar) - 1;
    if (ixlast >= 1) {
      iy = obj.nvar;
      i = ixlast - 1;
      scalarLB = (ixlast / 2) << 1;
      vectorUB = scalarLB - 2;
      for (int k{0}; k <= vectorUB; k += 2) {
        ixlast = iy + k;
        r = _mm_loadu_pd(&obj.grad.data[ixlast]);
        _mm_storeu_pd(&obj.grad.data[ixlast],
                      _mm_add_pd(r, _mm_set1_pd(obj.rho)));
      }
      for (int k{scalarLB}; k <= i; k++) {
        ixlast = iy + k;
        obj.grad.data[ixlast] += obj.rho;
      }
    }
  } break;
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeGrad_StoreHx.cpp
//
// [EOF]
//
