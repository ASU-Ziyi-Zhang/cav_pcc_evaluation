clc;
clear all;
close all;

default_folder_path = pwd; 
CAVmdl_folder_path = [default_folder_path,'\mdlCAV'];
cd(CAVmdl_folder_path)

%%
% generate a mex function

% simple example
% codegen mcadd -args {[0 0 0 0],0}

% CAV mdl example
IntscInfo = zeros(6,5);
SpdLimInfo = zeros(2,2);
CtrlInfo = zeros(1,14);
CtrlPar = zeros(1,15);
StopInfo = zeros(1,2);

cfg = coder.config('mex');
codegen -config cfg CAV_mdl -args {IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo}

%%
% generate a dynamic C/C++ library (.dll) with a package
cfg = coder.config('dll');
cfg.TargetLang = 'C'; % Libraries in Python need C bindings - extern _declspec(dllexport)
cfg.Verbosity = 'Verbose';
cfg.GenCodeOnly = true; % True or false - if True use CMake on target hardware to generate the DLL/SO

codegen -config cfg -package CAV_mdl -args {IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo}

codebuild('./codegen/dll/CAV_ctrl_mdl_240808', 'BuildMethod', 'CMake', 'BuildVariant', 'SHARED_LIBRARY'); % BuildMethod CMake creates a CMakeLists.txt
packNGo('./codegen/dll/CAV_ctrl_mdl_240808');

%%% See also https://blogs.mathworks.com/simulink/2022/12/23/leveraging-generated-code-from-matlab-in-a-c-application/
% 
% C:\work> cd codegen\dll\CAV_ctrl_mdl_240808
% From the directory that contains the CMakeLists.txt:
%{
    C:\work> mkdir build
    C:\work> cmake -B ./build -DCMAKE_BUILD_TYPE=Release
    C:\work> cmake --build build --config Release
    C:\work> cmake --install build
%}