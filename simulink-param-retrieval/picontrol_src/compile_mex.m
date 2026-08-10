
% compile_mex.m
% Version: 2.1
% Compiles read_params_sfcn MEX for Simulink desktop simulation.
% Run once per model (to pick up that model's params_config.h).
% Not needed for Pi deployment — Coder uses the .c source directly.
%
% Usage:
%   1. Open MATLAB
%   2. cd to picontrol_src/
%   3. Edit MODEL_CONFIG_DIR below to point to your model folder
%   4. Run: compile_mex
%
% --- Edit to point to your model folder ---
MODEL_CONFIG_DIR_NAME = 'TestCartPend1'; % No leading / !!
% ---------------------------------------------
[CURRENT_DIR, ~, ~] = fileparts(mfilename('fullpath'));
disp(['Working directory: ', CURRENT_DIR])
CURRENT_DIR      = fullfile(CURRENT_DIR, '..');
SRC_DIR          = fullfile(CURRENT_DIR, '/picontrol_src');
MODEL_CONFIG_DIR = fullfile(CURRENT_DIR, MODEL_CONFIG_DIR_NAME)
% ---------------------------------------------

fprintf('Compiling read_params_sfcn...\n');
fprintf('  src:    %s\n', SRC_DIR);
fprintf('  config: %s\n', MODEL_CONFIG_DIR);

mex(fullfile(SRC_DIR, 'read_params_sfcn.c'), ...
    fullfile(SRC_DIR, 'cJSON.c'), ...
    ['-I' SRC_DIR], ...
    ['-I' MODEL_CONFIG_DIR]);

fprintf('[compile_mex] Done.\n');