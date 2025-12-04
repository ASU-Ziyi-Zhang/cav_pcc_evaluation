//
// File: setProblemType.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "setProblemType.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "modifyOverheadPhaseOne_.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : j_struct_T &obj
//                int PROBLEM_TYPE
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(j_struct_T &obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj.nVar = obj.nVarOrig;
    obj.mConstr = obj.mConstrOrig;
    if (obj.nWConstr[4] > 0) {
      int idxUpperExisting;
      idxUpperExisting = obj.isActiveIdx[4] - 2;
      i = static_cast<unsigned char>(obj.sizesNormal[4]);
      for (int idxStartIneq{0}; idxStartIneq < i; idxStartIneq++) {
        int i1;
        i1 = (idxUpperExisting + idxStartIneq) + 1;
        obj.isActiveConstr.data[(obj.isActiveIdxNormal[4] + idxStartIneq) - 1] =
            obj.isActiveConstr.data[i1];
        obj.isActiveConstr.data[i1] = false;
      }
    }
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxNormal[i];
    }
  } break;
  case 1:
    obj.nVar = obj.nVarOrig + 1;
    obj.mConstr = obj.mConstrOrig + 1;
    for (int i{0}; i < 5; i++) {
      obj.sizes[i] = obj.sizesPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    for (int i{0}; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxPhaseOne[i];
    }
    break;
  case 2: {
    int i;
    obj.nVar = obj.nVarMax - 1;
    obj.mConstr = obj.mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegularized[i];
    }
    if (obj.probType != 4) {
      int i1;
      int i2;
      int idxStartIneq;
      int idxUpperExisting;
      int offsetIneq_tmp;
      offsetIneq_tmp = obj.nVarOrig + 1;
      i = static_cast<unsigned char>(obj.sizes[0]);
      for (int idx_col{0}; idx_col < i; idx_col++) {
        idxUpperExisting = obj.ldA * idx_col;
        i1 = obj.nVar;
        if (offsetIneq_tmp <= i1) {
          std::memset(
              &obj.ATwset.data[(offsetIneq_tmp + idxUpperExisting) + -1], 0,
              static_cast<unsigned int>(
                  (((i1 + idxUpperExisting) - offsetIneq_tmp) -
                   idxUpperExisting) +
                  1) *
                  sizeof(double));
        }
      }
      i = static_cast<unsigned char>(obj.sizes[2]);
      for (int idx_col{0}; idx_col < i; idx_col++) {
        idxUpperExisting = obj.ldA * idx_col - 1;
        i1 = offsetIneq_tmp + idx_col;
        i2 = i1 - 1;
        if (offsetIneq_tmp <= i2) {
          std::memset(&obj.Aineq.data[offsetIneq_tmp + idxUpperExisting], 0,
                      static_cast<unsigned int>(
                          (((i2 + idxUpperExisting) - offsetIneq_tmp) -
                           idxUpperExisting) +
                          1) *
                          sizeof(double));
        }
        obj.Aineq.data[i1 + idxUpperExisting] = -1.0;
        i1++;
        i2 = obj.nVar;
        if (i1 <= i2) {
          std::memset(
              &obj.Aineq.data[i1 + idxUpperExisting], 0,
              static_cast<unsigned int>(
                  (((i2 + idxUpperExisting) - i1) - idxUpperExisting) + 1) *
                  sizeof(double));
        }
      }
      idxUpperExisting = obj.nVarOrig;
      i = obj.sizesNormal[3] + 1;
      i1 = obj.sizesRegularized[3];
      for (idxStartIneq = i; idxStartIneq <= i1; idxStartIneq++) {
        idxUpperExisting++;
        obj.indexLB.data[idxStartIneq - 1] = idxUpperExisting;
      }
      if (obj.nWConstr[4] > 0) {
        i = static_cast<unsigned char>(obj.sizesRegularized[4]);
        for (idxStartIneq = 0; idxStartIneq < i; idxStartIneq++) {
          obj.isActiveConstr
              .data[obj.isActiveIdxRegularized[4] + idxStartIneq] =
              obj.isActiveConstr.data[(obj.isActiveIdx[4] + idxStartIneq) - 1];
        }
      }
      i = obj.isActiveIdx[4];
      i1 = obj.isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        std::memset(&obj.isActiveConstr.data[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) *
                        sizeof(boolean_T));
      }
      i = obj.nVarOrig + obj.sizes[2];
      if (offsetIneq_tmp <= i) {
        std::memset(&obj.lb.data[offsetIneq_tmp + -1], 0,
                    static_cast<unsigned int>((i - offsetIneq_tmp) + 1) *
                        sizeof(double));
      }
      idxStartIneq = obj.isActiveIdx[2];
      i = obj.nActiveConstr;
      for (int idx_col{idxStartIneq}; idx_col <= i; idx_col++) {
        idxUpperExisting = obj.ldA * (idx_col - 1) - 1;
        if (obj.Wid.data[idx_col - 1] == 3) {
          i1 = offsetIneq_tmp + obj.Wlocalidx.data[idx_col - 1];
          i2 = i1 - 2;
          if (offsetIneq_tmp <= i2) {
            std::memset(&obj.ATwset.data[offsetIneq_tmp + idxUpperExisting], 0,
                        static_cast<unsigned int>(
                            (((i2 + idxUpperExisting) - offsetIneq_tmp) -
                             idxUpperExisting) +
                            1) *
                            sizeof(double));
          }
          obj.ATwset.data[(i1 + idxUpperExisting) - 1] = -1.0;
          i2 = obj.nVar;
          if (i1 <= i2) {
            std::memset(
                &obj.ATwset.data[i1 + idxUpperExisting], 0,
                static_cast<unsigned int>(
                    (((i2 + idxUpperExisting) - i1) - idxUpperExisting) + 1) *
                    sizeof(double));
          }
        } else {
          i1 = obj.nVar;
          if (offsetIneq_tmp <= i1) {
            std::memset(&obj.ATwset.data[offsetIneq_tmp + idxUpperExisting], 0,
                        static_cast<unsigned int>(
                            (((i1 + idxUpperExisting) - offsetIneq_tmp) -
                             idxUpperExisting) +
                            1) *
                            sizeof(double));
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegularized[i];
    }
  } break;
  default:
    obj.nVar = obj.nVarMax;
    obj.mConstr = obj.mConstrMax;
    for (int i{0}; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegPhaseOne[i];
    }
    modifyOverheadPhaseOne_(obj);
    for (int i{0}; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegPhaseOne[i];
    }
    break;
  }
  obj.probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for setProblemType.cpp
//
// [EOF]
//
