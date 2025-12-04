////////////////////////////////
/// Tyler Ard                ///
/// Argonne National Lab     ///
/// Vehicle Mobility Systems ///
/// tard(at)anl.gov          ///
////////////////////////////////

#include "pccmpc_api.h" // File from autogen

/* External API bindings */

// Object constructors
DLL_PORTING AUTOGENCLASS* new_controller(){ return new AUTOGENCLASS(); }
DLL_PORTING AUTOGENINPUT* new_inputs(){ return new AUTOGENINPUT(); }
DLL_PORTING AUTOGENOUTPUT* new_outputs(){ return new AUTOGENOUTPUT(); }

// Object methods
DLL_PORTING void init_controller(AUTOGENCLASS* controller){ controller->initialize(); }

DLL_PORTING void set_inputs(AUTOGENCLASS* controller, AUTOGENINPUT* inputs){ controller->setExternalInputs(inputs); }
DLL_PORTING void step_controller(AUTOGENCLASS* controller){ controller->step(); }
DLL_PORTING void get_outputs(AUTOGENCLASS* controller, AUTOGENOUTPUT* outputs){ *outputs = controller->getExternalOutputs(); }

// Object destructors
DLL_PORTING void cleanup_controller(AUTOGENCLASS* controller){ 
    // Cleanup code gen model
    controller->terminate();

    // Free memory
    delete controller;
}
DLL_PORTING void cleanup_inputs(AUTOGENCLASS* inputs){ delete inputs; }
DLL_PORTING void cleanup_outputs(AUTOGENCLASS* outputs){ delete outputs; }