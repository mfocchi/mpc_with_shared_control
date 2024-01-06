clc; clear all;


USEGENCODE = true; % need to run gen_cpp_code_mpc


% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 

params.model = 'UNICYCLE';

params.obstacle_avoidance = false;
params.mpc_N = 30;
params.omega_max = 5.;
params.omega_min = -5.;
params.v_max = 1;
params.v_min = -1;
params.obstacle_pos = [0.5; 0.25];
params.obstacle_radius = 0.2;

params.DEBUG_COST = false;
params.mpc_dt = 0.1;
constr_tolerance = 1e-3;
dt=0.1; % only to evaluate solution

v_d = 0.1;
omega_d= 0.1;
sim_duration = 20;
v_vec0 = zeros(1,params.mpc_N);
omega_vec0 = zeros(1,params.mpc_N);


params.int_method = 'rk4';
params.int_steps = 5.; %0 means normal intergation
params.w1 =1; % tracking x
params.w2 =2; % tracking x
params.w3= 0.1; % tracking theta
params.w4= 0.01; % smooth term 

[ref,ref_time]  = genReference(p0, v_d, omega_d, dt, sim_duration/dt);

samples = length(ref) - params.mpc_N+1;
start_mpc = 1;
actual_t = ref_time(start_mpc);
actual_state = p0;

%ADD NOISE ON state
actual_state = actual_state +[0.05;0.1; 0.1]; %make the disturbance suvh that the robot is pulling
  
mpc_fun   = 'optimize_cpp_mpc';
if USEGENCODE
    mpc_fun=append(mpc_fun,'_mex' );
end
mpc_fun_handler = str2func(mpc_fun);

%gen code if needed
if ~isfile('optimize_cpp_mpc_mex.mexa64')
    disp('Generating C++ code');
    cfg = coder.config('mex');  
    coder.cstructname(params, 'params');
    codegen -config cfg  optimize_cpp_mpc -args { zeros(3,1), 0,  coder.typeof(1,[3 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) , coder.cstructname(params, 'params') } -nargout 1 -report 
end


%

for i=start_mpc:samples
    local_ref = ref(:,i:i+params.mpc_N-1);      
    fprintf("Iteration #: %d\n", i)
    
    solution = mpc_fun_handler(actual_state, actual_t, local_ref, v_vec0, omega_vec0, params);
    % 
    % 1 First-order optimality measure was less than options.OptimalityTolerance, and maximum constraint violation was less than options.ConstraintTolerance.
    % 0 Number of iterations exceeded options.MaxIterations or number of function evaluations exceeded options.MaxFunctionEvaluations.
    % -1 Stopped by an output function or plot function.
    % -2 No feasible point was found.
    % 2 Change in x was less than options.StepTolerance and maximum constraint violation was less than options.ConstraintTolerance.

    switch solution.problem_solved
        case 1 
            fprintf(2,"Problem converged!\n")
        case -2  
            fprintf(2,"Problem didnt converge!\n")
        case 2 
            fprintf(2,"semidefinite solution (should modify the cost)\n")
        case 0 
            fprintf(2,"Max number of feval exceeded (10000)\n")
    end    
    
    
    v = solution.x(1:params.mpc_N);
    omega = solution.x(params.mpc_N+1:2*params.mpc_N);
        

    %update dynamics (this emulates the real dynamics with noise)
    [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, params.mpc_dt/(params.int_steps-1), params.int_steps, v(1)*ones(1,params.int_steps),  omega(1)*ones(1,params.int_steps), params); 

    %plot
    clf(gcf)
    set(0, 'DefaultAxesBox', 'on');
    set(0, 'DefaultTextFontSize', 30);
    set(0, 'DefaultAxesFontSize', 30);
    set(0, 'DefaultUicontrolFontSize', 30);    


    % plot states
    subplot(3,2,1)    
    plot(ref_time(start_mpc:end), ref(1, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(1,:), 'bo-'); grid on;hold on; ylabel('X')
    
    subplot(3,2,3)       
    plot(ref_time(start_mpc:end), ref(2, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(2,:), 'bo-'); grid on;hold on; ylabel('Y')
    
    subplot(3,2,5)   
    plot(ref_time(start_mpc:end), ref(3, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(3,:), 'bo-'); grid on;hold on;  ylabel('theta')
    
    subplot(3,2,2)           
    plot(solution.mpc_time, v, 'go-'); grid on;hold on;  ylabel('v'); grid on;hold on;  
    xlim([min(solution.mpc_time), max(solution.mpc_time)]);
        plot(solution.mpc_time, ones(1,length(solution.mpc_time))*params.v_min, 'r'); grid on;hold on;  
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*(params.v_max), 'r'); grid on;hold on;  

    legend({'v','min', 'max'});

    subplot(3,2,4)   
    plot(solution.mpc_time, omega, 'bo-'); grid on;hold on;  ylabel('omega'); grid on;hold on;
    xlim([min(solution.mpc_time), max(solution.mpc_time)]);
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*params.omega_min, 'r'); grid on;hold on;  
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*(params.omega_max), 'r'); grid on;hold on;  
    legend({'omega','min', 'max'});

       
    pause(0.3)

    
     
end

