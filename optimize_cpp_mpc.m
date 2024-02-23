

function [solution] = optimize_cpp_mpc(actual_state, actual_t, obstacle_pos_x, obstacle_pos_y, local_ref, local_human_ref,  v_vec0, omega_vec0,  params)

        constr_tolerance = 1e-3;

        x0 = [ v_vec0,  omega_vec0];

        lb = [ params.v_min*ones(1,params.mpc_N),  params.omega_min*ones(1,params.mpc_N)];
        ub = [ params.v_max*ones(1,params.mpc_N),  params.omega_max*ones(1,params.mpc_N)];

        %%NON real time iteration
        options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
        'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance);
        % real time iteration
        %options = optimoptions('fmincon','Display','none','Algorithm','sqp',  ... % does not always satisfy bounds
        % 'MaxFunctionEvaluations', 10000, 'ConstraintTolerance',constr_tolerance,  'MaxIterations', 3);
        
        [x, final_cost, EXITFLAG, output] = fmincon(@(x) cost_mpc(x, actual_state, actual_t, local_ref, local_human_ref,  params),  x0,[],[],[],[],lb,ub, @(x) constraints_mpc(x, actual_state,  actual_t,obstacle_pos_x, obstacle_pos_y, params), options);
       
        
        % predict new state vector with the optimal v, omega profiles
        v =  x(1:params.mpc_N);
        omega = x(params.mpc_N+1:2*params.mpc_N);
        [mpc_states, mpc_time] = computeRollout(actual_state, actual_t,params.mpc_dt, params.mpc_N, v, omega, params);            

               
        % evaluate constraint violation 
        [c ceq] = constraints_mpc(x, actual_state,  actual_t,obstacle_pos_x, obstacle_pos_y, params);
     
        % init struct foc C++ code generation
        solution = struct;        
        solution.x = x;
        solution.v = v;
        solution.omega = omega;
        solution.cost = final_cost;
        solution.problem_solved = EXITFLAG ;%(EXITFLAG == 1) || (EXITFLAG == 2);
        % 1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
        % 0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations.
        % -1 Stopped by an output function or plot function.
        % -2 No feasible point was found.
        % 2 Change in x was less than options.StepTolerance (Termination tolerance on x, a scalar, the default is 1e-10) and maximum constraint violation was less than options.ConstraintTolerance.
  
        solution.mpc_states = mpc_states;
        solution.mpc_time = mpc_time;
        solution.optim_output = output;
        solution.c = c;    
        
end
