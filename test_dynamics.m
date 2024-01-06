close all 
clear all 

params.model = 'UNICYCLE'
params.int_method = 'rk4';
params.int_steps = 5;
N_dyn = 100;


%initial state
x0 =  [0.;0.; 0.];
dt = 0.01;
Tf =  10;
dt_dyn = Tf / (N_dyn-1);
%inputs
n_sim_steps = floor(Tf/dt);

v = 0.1;
omega = 0.2;

v_vec = ones(1, n_sim_steps) * v;
omega_vec = ones(1, n_sim_steps) * omega;



[~,~,x, t] = integrate_dynamics(x0, 0, dt, n_sim_steps, v_vec, omega_vec, params);


final_point = [x(1,end), x(2,end), x(3,end)];
ode45_final_point =[ -0.0000,    1.6240,   -1.6246];
fprintf('Compare with ode45 final point [%3.4f, %3.4f, %3.4f] \n', final_point- ode45_final_point)



figure
subplot(3,1,1)
plot(t, x(1,:) ,'-ro'); 
grid on;
subplot(3,1,2)
plot(t, x(2,:) ,'-ro');
grid on;
subplot(3,1,3)
plot(t, x(3,:) ,'-ro');
grid on;
figure
plot(x(1,:), x(2,:))
grid on
axis equal



% integrate with substeps (every Ndyn/int_steps)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v_rough = ones(1, N_dyn) * v;
omega_rough = ones(1, N_dyn) * omega;
[states_rough_sub, t_rough_sub] = computeRollout(x0, 0,dt_dyn, N_dyn, v_rough, omega_rough, params);
 
% integrate without substep (every Ndyn) 
params.int_steps = 0;
[states_rough, t_rough] = computeRollout(x0, 0,dt_dyn, N_dyn, v_rough, omega_rough, params);


figure
subplot(3,1,1)
plot(t_rough, states_rough(1,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(1,:),'ko'); hold on;grid on;
plot(t, x(1,:),'-r'); 
legend('rough','rough sub','cont');
ylabel('X')

subplot(3,1,2)
plot(t_rough, states_rough(2,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(2,:),'ko'); hold on;grid on;
plot(t, x(2,:),'-r');
ylabel('Y')

subplot(3,1,3)
plot(t_rough, states_rough(3,:),'bo'); hold on
plot(t_rough_sub, states_rough_sub(3,:),'ko'); hold on; grid on;
plot(t, x(3,:),'-r');
ylabel('theta')


