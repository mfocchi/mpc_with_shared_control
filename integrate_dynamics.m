function [x_, t_, x_vec,  t_vec] = integrate_dynamics(x0, t0, dt, n_steps, v, omega, params)
    
    %verify is a column vector
    x0 = x0(:);
    t_ = t0;
    x_ = x0;
    x_vec = x0;
    t_vec = t_; 
 
    if strcmp(params.int_method, 'eul') % euler
            % forwatd euler
            for i=1:n_steps-1               
                x_ = x_ + dt* dynamics(t_, x_, v(i), omega(i),  params); % we have time invariant dynamics so t wont count
                t_ = t_ + dt;
                x_vec = [x_vec x_];
                t_vec = [t_vec t_];
            end
    elseif  strcmp(params.int_method, 'rk4') % rk4
            %https://www.geeksforgeeks.org/runge-kutta-4th-order-method-solve-differential-equation/
            h = dt;
            F = @(t, x, u1, u2) dynamics(t, x, u1, u2, params); % we have  time invariant dynamics so t wont count
            for i=1:n_steps-1                 
                k_1 = F(t_      , x_           ,v(i), omega(i));
                k_2 = F(t_+0.5*h, x_+ 0.5*h*k_1,v(i), omega(i));
                k_3 = F(t_+0.5*h, x_+ 0.5*h*k_2,v(i), omega(i));
                k_4 = F(t_+h    , x_+ k_3*h    ,v(i), omega(i));
                x_ = x_ + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;  
                t_ = t_ + h;
                x_vec = [x_vec x_];
                t_vec = [t_vec t_];
            end
    else  
            disp('Unknown method.')
    end  

end