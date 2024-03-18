clear all;
global params
addpath('/home/mfocchi/matlab/MPC_optimization/matlab_msg_gen_ros1/glnxa64/install/m')
pyenv

rosshutdown
rosinit

masterHost = 'localhost';
node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);


params.model = 'UNICYCLE';
params.obstacle_avoidance = true;
params.mpc_N = 15; %better
%store it into param server for the husky
rosparam("set","mpc_N",int32(params.mpc_N));
params.omega_max = 1.;
params.omega_min = -1.;
params.v_max = 2;
params.v_min = -2;
obstacle_pos = [0.4; 0.07];
params.obstacle_radius = 0.6;
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
params.w1 =10; % tracking x
params.w2 =5; % tracking y
params.w3= 10; % tracking theta
params.w4= 0.01; % smooth term 
params.w5= 1; % lin speed term (fundamental to avoid get stuck)
params.w6= 1e-05; % lin speed term (fundamental to avoid get stuck)
params.w7= 0; % shared control weight
% gen messages (only once)
%genDir = fullfile(pwd,'customMessages');
%%%%gen messages
% rosgenmsg
% clear classes
% rehash toolboxcache

%gen code (run if you did some change in the cost)
if ~isfile('optimize_cpp_mpc_mex.mexa64')
    disp('Generating C++ code');
    cfg = coder.config('mex');  
    coder.cstructname(params, 'params');
    codegen -config cfg  optimize_cpp_mpc -args { zeros(3,1), 0, 0, 0,  coder.typeof(1,[3 Inf]),coder.typeof(1,[2 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) ,0,0, coder.cstructname(params, 'params') } -nargout 1 -report 
end

server = rossvcserver('/mpc', 'customMessages/mpc', @MPCcallback,'DataFormat','struct');
                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%client
p0 = [0.0; 0.0; 0.]; 
[ref_state,ref_time]  = genReference(p0, params.v_d, params.omega_d, dt, sim_duration/dt);
[ref_pitch, ref_roll] = genHumanInput(0.05, -0.7, 1, 3,  1., dt, sim_duration/dt);

samples = length(ref_state) - params.mpc_N+1;
start_mpc = 1;
actual_t = ref_time(start_mpc);
actual_state = p0;

%log vectors
log_state = actual_state;
log_controls = [];
log_human_ref = [];
log_time = 0;


local_ref = ref_state(:,1:1+params.mpc_N-1);  
local_human_ref = [ref_pitch(:,1:1+params.mpc_N-1); ref_roll(:,1:1+params.mpc_N-1)];

mpcclient = rossvcclient("/mpc","DataFormat","struct")
mpcreq = rosmessage(mpcclient);
%msg = rosmessage('geometry_msgs/Vector3')
%msg = rosmessage( 'std_msgs/Float32MultiArray')
%rosmsg show std_msgs/Float64

mpcreq.ActualState.X = actual_state(1);
mpcreq.ActualState.Y = actual_state(2);
mpcreq.ActualState.Z = actual_state(3);

singleMsg = rosmessage( 'std_msgs/Float64',"DataFormat","struct");
poseMsg = rosmessage("geometry_msgs/Vector3","DataFormat","struct");
for k = 1:params.mpc_N
    poseMsg.X = local_ref(1,k);
    poseMsg.Y = local_ref(2,k);
    poseMsg.Z = local_ref(3,k);
    mpcreq.LocalRef(k) = poseMsg;  
end
singleMsg.Data = actual_t;
mpcreq.ActualTime = singleMsg;
singleMsg.Data = local_human_ref(1,1);
mpcreq.Roll = singleMsg;
singleMsg.Data = local_human_ref(2,1);
mpcreq.Pitch = singleMsg;

singleMsg.Data = params.v_d;
mpcreq.DesV = singleMsg;
singleMsg.Data = params.omega_d;
mpcreq.DesOmega = singleMsg;

mpcreq.ObstaclePos.X = obstacle_pos(1);
mpcreq.ObstaclePos.Y = obstacle_pos(2);


%display message content
%rosShowDetails(mpcreq)
if isServerAvailable(mpcclient)
    tic
    mpcresp = call(mpcclient,mpcreq, "Timeout",3)
    toc
else
    error("Service server not available on network")
end

mpcresp.LinVel.Data
mpcresp.AngVel.Data

% 
