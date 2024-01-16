function [pitch, roll] = genHumanInput(max_pitch, max_roll, pitch_time_instant, roll_time_instant,  dist_duration, dT, horizon_length)
 
   
   time = 0;
   pitch = zeros(1, horizon_length);   
   roll = zeros(1, horizon_length);
   
   for i=1:horizon_length 
        
       if (time >= roll_time_instant)  && (time < (roll_time_instant+dist_duration))
           roll(i) =max_roll ;
     
       end
       
       if (time >= pitch_time_instant)  && (time < (pitch_time_instant+dist_duration))
           pitch(i) = max_pitch;
     
       end
       time = time + dT;
       
       
   end
    %append one element cause there are N+1 elements
   roll = [roll 0];
   pitch = [ pitch 0];
end