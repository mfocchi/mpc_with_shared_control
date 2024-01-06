function dxdt = unicycle_model(x, v_input, omega_input, params)
    theta = x(3);    

    dxdt = [v_input*cos(theta); v_input*sin(theta); omega_input]; 
end

