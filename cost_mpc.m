function cost = cost_mpc(x, state0,  actual_t, local_ref, local_human_ref,  params)

    local_ref_pitch = local_human_ref(1,:);
    local_ref_roll = local_human_ref(2,:);

    
    % init for cpp
    ref_mpc = zeros(3,params.mpc_N);
    cost=0;
    
    
    if (length(local_ref) < params.mpc_N) 
        disp('cost_mpc:wrong ref_com input length: ref_com should be longer than mpc_N')
        local_ref
        return
    end
    
    v_vec = x(1:params.mpc_N); 
    omega_vec = x(params.mpc_N+1:2*params.mpc_N); 

    % compute actual state           
    [states, t] = computeRollout(state0, actual_t, params.mpc_dt, params.mpc_N, v_vec, omega_vec, params);
    
    %p has mpc_N +1 elements 
    % cartesian track
    norm_order = 2;
    tracking_x= sum (vecnorm(local_ref(1,:)- states(1,:), norm_order).^2);    
    tracking_y= sum (vecnorm(local_ref(2,:)- states(2,:), norm_order).^2);    
    tracking_theta= sum (vecnorm(local_ref(3,:) - states(3,:), norm_order).^2); 
    tracking_lin_speed= sum ( vecnorm( params.v_d*ones(1,params.mpc_N) -  v_vec, norm_order).^2 ); 
   
    % smoothnes: minimize jerky control action
    smoothing_speed = sum(diff(v_vec).^2)+ sum(diff(omega_vec).^2);  
    % minimize input
    effort = sum((v_vec).^2)+ sum((omega_vec).^2);  

    %shared control
    %compute activation function (scalar) with first velocity value
    %use this  in real implementation 
    %pitch_error = local_ref_pitch(1) - prev_v;
    %roll_error = local_ref_roll(1) - prev_omega; 
    
    if local_ref_pitch(1) ~=0 
        pitch_error = local_ref_pitch(1) - params.v_d;
    else 
        pitch_error = 0;
    end
    if local_ref_roll(1) ~=0 
        roll_error = local_ref_roll(1) -  params.omega_d;
    else 
        roll_error = 0;
    end
     
    Vh_v = params.ks * (pitch_error )^2;
    Vh_omega = params.ks* (roll_error)^2;
    gain_v = 1-exp(-Vh_v);   
    gain_omega = 1-exp(-Vh_omega);   
    
    tracking_human_input = gain_v*(v_vec(1)-local_ref_pitch(1))^2 + gain_omega*(omega_vec(1) - local_ref_roll(1))^2; 
       
    
    cost_components = struct;
    %to avoid nan issue incorporate them in the cost only if the weight is
    %not zero!
    if  params.w1 ~=0 
        cost_components.x   =  params.w1 * tracking_x;
    else 
        cost_components.x   = 0;
    end
    
    if  params.w2 ~=0 
        cost_components.y   =  params.w2 * tracking_y;
    else 
        cost_components.y   = 0;
    end
    

    if  params.w3 ~=0     
        cost_components.theta   =  params.w3 *tracking_theta;
    else 
        cost_components.theta   = 0;
    end

    if params.w4~=0
        cost_components.smoothing_speed   =  params.w4 *smoothing_speed;
    else 
        cost_components.smoothing_speed   = 0;
    end

    if params.w5~=0
        cost_components.tracking_lin_speed   =  params.w5 *tracking_lin_speed;
    else 
        cost_components.tracking_lin_speed   = 0;
    end

    if params.w6~=0
        cost_components.input   =  params.w6 *effort;
    else 
        cost_components.input   = 0;
    end
    
   if params.w7~=0
        cost_components.human   =  params.w7 *tracking_human_input;
    else 
        cost_components.human   = 0;
    end
         
    if params.DEBUG_COST
      
        fprintf('cost components: x %f,  y %f, theta : %f smoothing : %f  lin_speed : %f input : %f  human %f \n \n',...
                        cost_components.x,cost_components.y,  cost_components.theta,  cost_components.smoothing_speed,  ...
                        cost_components.tracking_lin_speed, cost_components.input, cost_components.human);
    end
    cost =  cost_components.x  +  cost_components.y +  cost_components.theta +  cost_components.smoothing_speed  + ...
            cost_components.tracking_lin_speed + cost_components.input +cost_components.human;

end