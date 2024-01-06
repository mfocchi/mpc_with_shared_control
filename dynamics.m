function [dxdt] = dynamics(t, x, v, omega, params) % because we have time invariant system t wont be used
    switch (params.model)
        case 'UNICYCLE'
            dxdt = unicycle_model(x, v ,omega, params);        
      
        otherwise
            disp('wrong model')
            dxdt = [0;0;0];
    end
end


