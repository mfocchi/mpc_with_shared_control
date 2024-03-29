function resp = MPCcallback(~,req,resp)
global params
%disp('A service client is calling');
actual_state(1) = req.ActualState.X ; 
actual_state(2) = req.ActualState.Y ;
actual_state(3) = req.ActualState.Z ;

for k = 1:params.mpc_N
    local_ref(1,k)= req.LocalRef(k).X;
    local_ref(2,k)= req.LocalRef(k).Y;
    local_ref(3,k)= req.LocalRef(k).Z;
end
actual_t = req.ActualTime.Data;

v_vec0 = zeros(1,params.mpc_N);
omega_vec0 = zeros(1,params.mpc_N);

local_human_ref = [req.Pitch.Data*ones(1,params.mpc_N); req.Roll.Data*ones(1,params.mpc_N)];

des_v = req.DesV.Data;
des_omega = req.DesOmega.Data;
obstacle_pos(1) = req.ObstaclePos.X ; 
obstacle_pos(2) = req.ObstaclePos.Y ; 

%debug 
% actual_state
% local_ref(:,1)'

solution = optimize_cpp_mpc_mex(actual_state(:), actual_t, obstacle_pos(1), obstacle_pos(2), local_ref, local_human_ref, v_vec0, omega_vec0, des_v, des_omega, params);
resp.LinVel.Data = solution.v(1);
resp.AngVel.Data = solution.omega(1);
 

end