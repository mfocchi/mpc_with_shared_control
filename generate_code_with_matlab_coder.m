clc; clear all;
close all;

% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; 0.]; 

params.model = 'UNICYCLE';

params.obstacle_avoidance = true;
params.mpc_N = 30;
params.omega_max = 1.;
params.omega_min = -1.;
params.v_max = 0.5;
params.v_min = -0.5;
params.obstacle_pos = [0.4; 0.07];
params.obstacle_radius = 0.06;
params.ks =5;

params.DEBUG_COST = false;
params.mpc_dt = 0.1;
constr_tolerance = 1e-3;
dt=0.1; 

% desired surge speed /ang velocity
params.v_d = 0.1;
params.omega_d= 0.1;

sim_duration = 10;
v_vec0 = zeros(1,params.mpc_N);
omega_vec0 = zeros(1,params.mpc_N);

params.int_method = 'rk4';
params.int_steps = 5.; %0 means normal integration
params.w1 =1; % tracking x
params.w2 =10; % tracking y
params.w3= 0.1; % tracking theta
params.w4= 0.01; % smooth term 
params.w5= 1; % lin speed term (fundamental to avoid get stuck)
params.w6= 1e-05; % lin speed term (fundamental to avoid get stuck)
params.w7= 100; % shared control weight

[ref_state,ref_time]  = genReference(p0, params.v_d, params.omega_d, dt, sim_duration/dt);
[ref_pitch, ref_roll] = genHumanInput(-0.05, -0.7, 1, 3,  1., dt, sim_duration/dt);

samples = length(ref_state) - params.mpc_N+1;
start_mpc = 1;
actual_t = ref_time(start_mpc);
actual_state = p0;

%log vectors
log_state = actual_state;
log_controls = [];
log_human_ref = [];
log_time = 0;
prev_controls = [v_vec0;omega_vec0];
    
local_ref = ref_state(:,1:1+params.mpc_N-1);  
local_human_ref = [ref_pitch(:,1:1+params.mpc_N-1); ref_roll(:,1:1+params.mpc_N-1)];

solution = optimize_cpp_mpc(actual_state, actual_t, local_ref, local_human_ref, prev_controls, v_vec0, omega_vec0, params);

% 
disp('Generating C++ code');
%cfg = coder.config('lib');  % static library .a
cfg = coder.config('dll');  % dynamic library .so
cfg.GenerateExampleMain = 'GenerateCodeOnly';
cfg.TargetLang = 'C++'; %default is C
coder.cstructname(params, 'params');
codegen -config cfg  optimize_cpp_mpc -args { zeros(3,1), 0,  coder.typeof(1,[3 Inf]),coder.typeof(1,[2 Inf]), coder.typeof(1,[2 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) , coder.cstructname(params, 'params') } -nargout 1 -report 

myBuildInfoFile = 'codegen/dll/optimize_cpp_mpc/buildInfo.mat';
load(myBuildInfoFile); %  loads data from the buildInfo.mat file in buildFolder and creates buildInfo variable
% 

%copile a shared dynamic library 'STANDALONE_EXECUTABLE' is creating an executable
%and is not what we want
%buildResults = codebuild(myBuildInfoFile, 'BuildVariant', 'SHARED_LIBRARY')

% %create CmakeLists.txt
codebuild(myBuildInfoFile, 'BuildMethod','CMake')


% % creates zip file
packNGo(myBuildInfoFile,'packType', 'hierarchical', 'nestedZipFiles', false, 'fileName','mpc_test'); %manually copy cmakelists.txt inside codegen/dll/optimize_cpp_mpc main.cpp and main.h


% 
movefile ./codegen/dll/optimize_cpp_mpc/examples/main.cpp
movefile ./codegen/dll/optimize_cpp_mpc/examples/main.h
