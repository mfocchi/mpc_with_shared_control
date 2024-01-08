function [ref, time] = generateReference(x0, v_d, omega_d,dT, horizon_length)

   x_ref = x0(1);
   y_ref = x0(2);
   theta_ref = x0(3);

   ref =[x_ref; y_ref;theta_ref ];

   %compute vd omegad from pf
   % Tf = dT*horizon_length;   
   % omega_d = (xf(3) - x0(3))/Tf;
   % Rz=@(angle)[cos(angle)    -sin(angle) 0
   %          sin(angle)   cos(angle)   0
   %          0           0             1];    
   % robot_orient = Rz(theta_ref)*[1;0];
   % v_d = sign(dot(robot_orient, xf(1:2) - x0(1:2)))*norm(xf(1:2) - x0(1:2))/ (Tf);
  

   for i=1:horizon_length                
        x_ref = x_ref  + v_d*cos(theta_ref) * dT;
        y_ref = y_ref  + v_d*sin(theta_ref) * dT;
        theta_ref =theta_ref + omega_d*dT;        
        ref = [ref , [x_ref; y_ref;theta_ref ]];

   end
   %append one element cause there are N+1 elements
   time = [1:horizon_length]*dT;
   time = [0 time];
end