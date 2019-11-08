% This file is created by U. Cem Kaya - 2018
function V_approaching = closingSpeed(r_target_next,r_UAV,LOS_angle,t,delta_t)

persistent LOS_prev LOS_angle_prev

if t == 0
    LOS_current = Dist(r_target_next,r_UAV);
    LOS_prev = LOS_current;
    V_approaching = 0;
    LOS_angle_prev = LOS_angle;
else
    
    
    LOS_current = Dist(r_target_next,r_UAV);

%     delta_x = LOS_current*cos(LOS_angle)-LOS_prev*cos(LOS_angle_prev);
%     delta_y = LOS_current*sin(LOS_angle)-LOS_prev*sin(LOS_angle_prev);
    
%     V_approaching = sqrt(delta_x^2 + delta_y^2)/delta_t;
    V_approaching = (LOS_current-LOS_prev)/delta_t;

    LOS_prev = LOS_current;
    LOS_angle_prev = LOS_angle;
    
end

end