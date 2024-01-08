clc; clear all;
close all;

USEGENCODE = true; % need to run gen_cpp_code_mpc


% INITIAL STATE (X,Y, THETA)
p0 = [0.0; 0.0; -0.]; 

params.model = 'UNICYCLE';

params.obstacle_avoidance = true;
params.mpc_N = 30;
params.omega_max = 5.;
params.omega_min = -5.;
params.v_max = 1;
params.v_min = -1;
params.obstacle_pos = [0.4; 0.07];
params.obstacle_radius = 0.06;
params.ks =1;

params.DEBUG_COST = false;
params.mpc_dt = 0.1;
constr_tolerance = 1e-3;
dt=0.1; % only to evaluate solution

params.v_d = 0.1;
params.omega_d= 0.1;
sim_duration = 10;
v_vec0 = zeros(1,params.mpc_N);
omega_vec0 = zeros(1,params.mpc_N);


params.int_method = 'rk4';
params.int_steps = 5.; %0 means normal intergation
params.w1 =1; % tracking x
params.w2 =10; % tracking y
params.w3= 0.1; % tracking theta
params.w4= 0.01; % smooth term 
params.w5= 1; % lin speed term (fundamental to avoid get stuck)
params.w6= 1e-05; % lin speed term (fundamental to avoid get stuck)
params.w7= 0; % shared control

[ref_state,ref_time]  = genReference(p0, params.v_d, params.omega_d, dt, sim_duration/dt);
[ref_pitch, ref_roll] = genHumanInput(0.5, 0., 1, 2,  0.5, dt, sim_duration/dt);

samples = length(ref_state) - params.mpc_N+1;
start_mpc = 1;
actual_t = ref_time(start_mpc);
actual_state = p0;

%ADD NOISE ON state
actual_state = actual_state ;%+[0.05;0.1; 0.1]; %make the disturbance suvh that the robot is pulling
  
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
    codegen -config cfg  optimize_cpp_mpc -args { zeros(3,1), 0,  coder.typeof(1,[3 Inf]),coder.typeof(1,[2 Inf]), coder.typeof(1,[2 Inf]), coder.typeof(1,[1 Inf]), coder.typeof(1,[1 Inf]) , coder.cstructname(params, 'params') } -nargout 1 -report 
end


%log vectors
log_state = actual_state;
log_controls = [];
log_human_ref = [];
log_time = 0;
prev_controls = [v_vec0;omega_vec0];

for i=start_mpc:samples
    local_ref = ref_state(:,i:i+params.mpc_N-1);  
    local_human_ref = [ref_pitch(:,i:i+params.mpc_N-1); ref_roll(:,i:i+params.mpc_N-1)];
    fprintf("Iteration #: %d\n", i)
    
    solution = mpc_fun_handler(actual_state, actual_t, local_ref, local_human_ref, prev_controls, v_vec0, omega_vec0, params);
   
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
    
    %DEBUG
%     fprintf(2,"number of iterations: %i\n", solution.optim_output.iterations);
%     fprintf(2,"number of func evaluations: %i\n", solution.optim_output.funcCount);
% 
%     [final_cost, cost_components] = cost(solution.x,actual_state, actual_t, local_ref, params);
%     constr_viol = solution.c>constr_tolerance;
%     num_constr_viol = sum(constr_viol);
%     if num_constr_viol>0
%         index_violated =find(constr_viol);
%         outputstr = repmat('%i ', 1, num_constr_viol); % replicate it to match the number of columns
%         fprintf(strcat('obstacle constraints violated at index:  ', outputstr,'\n'),index_violated);
%     else
%         fprintf('obstacle constraints violated:  %i\n\n',num_constr_viol);
%     end
%     fprintf('cost:  %f\n\n',final_cost);
%     fprintf('cost component: x: %f,  y : %f  theta : %f \n \n',cost_components.x,  cost_components.y, cost_components.theta);
%     
    
    v = solution.x(1:params.mpc_N);
    omega = solution.x(params.mpc_N+1:2*params.mpc_N);
    prev_controls = [v; omega];    

    %update dynamics (this emulates the real dynamics with noise)
    [actual_state, actual_t] = integrate_dynamics(actual_state ,actual_t, params.mpc_dt/(params.int_steps-1), params.int_steps, v(1)*ones(1,params.int_steps),  omega(1)*ones(1,params.int_steps), params); 

    log_state = [log_state actual_state];
    log_controls = [log_controls [v(1);omega(1)]];
    log_time = [log_time actual_t];    
    log_human_ref = [log_human_ref local_human_ref(:,1)];
    
    %plot
    clf(gcf)
    set(0, 'DefaultAxesBox', 'on');
    set(0, 'DefaultTextFontSize', 30);
    set(0, 'DefaultAxesFontSize', 30);
    set(0, 'DefaultUicontrolFontSize', 30);    


    % plot states
    subplot(3,2,1)    
    plot(ref_time(start_mpc:end), ref_state(1, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(1,:), 'bo-'); grid on;hold on; ylabel('X')
    plot(ref_time(i), ref_state(1, i), 'k.','MarkerSize', 20);
    ylim([0, max(ref_state(1,:))]);
    
    subplot(3,2,3)       
    plot(ref_time(start_mpc:end), ref_state(2, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(2,:), 'bo-'); grid on;hold on; ylabel('Y')
    plot(ref_time(i), ref_state(2, i), 'k.', 'MarkerSize', 20);
    ylim([0, max(ref_state(2,:))]);
    
    subplot(3,2,5)   
    plot(ref_time(start_mpc:end), ref_state(3, start_mpc:end), 'ro-'); grid on;hold on;
    plot(solution.mpc_time, solution.mpc_states(3,:), 'bo-'); grid on;hold on;  ylabel('theta')
    plot(ref_time(i), ref_state(3, i), 'k.','MarkerSize', 20);
    
    subplot(3,2,2)           
    plot(solution.mpc_time, v, 'go-'); grid on;hold on;  ylabel('v'); grid on;hold on;  
    xlim([min(solution.mpc_time), max(solution.mpc_time)]);
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*params.v_min, 'r'); grid on;hold on;  
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*(params.v_max), 'r'); grid on;hold on;
    plot(solution.mpc_time, local_human_ref(1,:),'ko-');
    legend({'v','min', 'max', 'pitch'});

    subplot(3,2,4)   
    plot(solution.mpc_time, omega, 'bo-'); grid on;hold on;  ylabel('omega'); grid on;hold on;
    xlim([min(solution.mpc_time), max(solution.mpc_time)]);
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*params.omega_min, 'r'); grid on;hold on;  
    plot(solution.mpc_time, ones(1,length(solution.mpc_time))*(params.omega_max), 'r'); grid on;hold on;  
    plot(solution.mpc_time, local_human_ref(2,:),'ko-');
    legend({'omega','min', 'max', 'roll'});

    
    subplot(3,2,6)   
    %initial REFERENCE
    plot(p0(1), p0(2) , 'Marker', '.', 'Color','g', 'MarkerSize',60) ; hold on; grid on;
    %initial ACTUAL
    plot(solution.mpc_states(1,1),  solution.mpc_states(2,1), 'bo-' , 'Marker', '.', 'Color','g', 'MarkerSize',60) ; hold on;
    % plot ref trajectory
    plot(ref_state(1,:), ref_state(2,:), 'ro-',   'LineWidth', 1.0,'MarkerSize',4);
    % plot actual trajectory
    plot(solution.mpc_states(1,:), solution.mpc_states(2,:), 'bo-',   'LineWidth', 1.0,'MarkerSize',4);
    plot(ref_state(1, i), ref_state(2, i), 'k.','MarkerSize', 20);
    % plot obstacle
    pos = params.obstacle_pos;
    r = params.obstacle_radius;
    th = linspace(0,2*pi*100);
    x = cos(th) ; y = sin(th) ;
    plot(pos(1) + r*x, pos(2) + r*y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2);hold on;
    plot(pos(1), pos(2), '.', 'MarkerSize', 50, 'LineWidth', 20);
    %initial orient
    plotOrientation([solution.mpc_states(1,1); solution.mpc_states(2,1)], solution.mpc_states(3,1), 0.02);
       
       
    pause(0.3);
    
     
end

plot_solution(log_time, log_state, ref_time, ref_state, log_controls, log_human_ref, p0, params, true);

