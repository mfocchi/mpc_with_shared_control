function [ineq, eq]= constraints_mpc(x, state0,  actual_t, params)
    ineq = [];
    eq = [];

    v_vec = x(1:params.mpc_N); 
    omega_vec = x(params.mpc_N+1:2*params.mpc_N); 
    
    % compute actual state           
    [states, t] = computeRollout(state0, actual_t, params.mpc_dt, params.mpc_N, v_vec, omega_vec, params);
    
    if params.obstacle_avoidance     
        
        % init for cpp  
        % size  known
        ineq = zeros(params.mpc_N,1);
   
        for i=1:params.mpc_N
            b =   (states(1:2,i)-params.obstacle_pos(:))'*(states(1:2,i)-params.obstacle_pos(:)) - params.obstacle_radius^2;

            % the quantity below should be <= 0
            ineq(i) = -b;
        end
    end
  
    
   
end