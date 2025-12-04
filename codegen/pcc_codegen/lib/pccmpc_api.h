////////////////////////////////
/// Tyler Ard                ///
/// Argonne National Lab     ///
/// Vehicle Mobility Systems ///
/// tard(at)anl.gov          ///
////////////////////////////////

#pragma once

#include "longitudinal_mpc.h" // File from autogen

// Typedefs of the name of the autogen model for convenience - Change these based on Simulink Model name
typedef longitudinal_mpc AUTOGENCLASS;
typedef ExtU_longitudinal_mpc_T AUTOGENINPUT;
typedef ExtY_longitudinal_mpc_T AUTOGENOUTPUT;

// External API bindings
#if defined(_WIN32) || defined(__LCC__) // Windows needs to also have a dllexport or dllimport declaration
#define DLL_PORTING __declspec(dllexport)
#else
#define DLL_PORTING
#endif

extern "C" {
    // Object constructors
    DLL_PORTING AUTOGENCLASS* new_controller();
    DLL_PORTING AUTOGENINPUT* new_inputs();
    DLL_PORTING AUTOGENOUTPUT* new_outputs();

    // Object methods
    DLL_PORTING void init_controller(AUTOGENCLASS* controller);

    DLL_PORTING void set_inputs(AUTOGENCLASS* controller, AUTOGENINPUT* inputs);
    DLL_PORTING void step_controller(AUTOGENCLASS* controller);
    DLL_PORTING void get_outputs(AUTOGENCLASS* controller, AUTOGENOUTPUT* outputs);
    
    // Object destructors
    DLL_PORTING void cleanup_controller(AUTOGENCLASS* controller);
    DLL_PORTING void cleanup_inputs(AUTOGENCLASS* inputs);
    DLL_PORTING void cleanup_outputs(AUTOGENCLASS* outputs);
}