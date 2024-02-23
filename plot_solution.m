function plot_solution(log_time, log_state, ref_time, ref_state, log_controls, log_human_ref,  p0, obstacle_pos, params, DEBUG)

 if nargin < 7
    DEBUG=false;
 end


if (DEBUG)
    
    
            
    figure
    title('states')
    subplot(3,1,1)    
    plot(log_time, log_state(1,:),'r') ; hold on;   grid on; 
    plot(ref_time, ref_state(1,:),'b') ; hold on;   grid on; 
    ylabel('X')
    
    subplot(3,1,2)
    plot(log_time, log_state(2,:),'r') ; hold on;  grid on;  
    plot(ref_time, ref_state(2,:),'b') ; hold on;   grid on;  
    ylabel('Y')
    
    subplot(3,1,3)
    plot(log_time, log_state(3,:),'r') ; hold on;  grid on;  
    plot(ref_time, ref_state(3,:),'b') ; hold on;   grid on;  
    ylabel('theta')
  
          
    figure
    title('controls')
    subplot(2,1,1)
    plot(log_time(2:end),params.v_max*ones(size(log_controls)),'r-'); hold on; grid on;
    plot(log_time(2:end),params.v_min*ones(size(log_controls)),'r-');
    plot(log_time(2:end), log_controls(1,:),'ob-') ; hold on; 
    plot(log_time(2:end) , log_human_ref(1,:), 'ok-');
    ylabel('v')    
    legend({'v','human'});
    
    subplot(2,1,2)
    plot(log_time(2:end),params.omega_max*ones(size(log_controls)),'r-'); hold on; grid on;
    plot(log_time(2:end),params.omega_min*ones(size(log_controls)),'r-');
    plot(log_time(2:end),log_controls(2,:),'ob-') ; hold on;    
    plot(log_time(2:end) , log_human_ref(2,:), 'ok-');
    ylabel('omega')
    legend({'omega','human'});
        
end

figure;
plot_curve(log_state, ref_state, p0, obstacle_pos, params, DEBUG)
shg

end