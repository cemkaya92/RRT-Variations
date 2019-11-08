% This file is created by U. Cem Kaya - 2018
function [UAV_path,UAV_location, UAV_angle_actual,UAV_speed_actual,w_comm,speed_comm,total_distance,RiskExposureAlongPath] =  ...
                    lineFollower(finalPath,figHandle,mapSize,solver_delta_t,V_tar1,total_distance_target,distThresh,PREM_value,dX,dY,PURM,X_Grid,Y_Grid,Population)


% start = [finalPath(1).x finalPath(1).y];
start = [finalPath(1).x finalPath(1).y];
finish = [finalPath(end).x finalPath(end).y];

delta_t = solver_delta_t;
output_sample_time = delta_t;

% initial Conditions for UAV
r_UAV = start;
% psi_UAV = 30*pi/180;
psi_UAV = finalPath(1).psi;
V_UAV = V_tar1;
W_UAV = 0;

V_comm = 0;
w_comm = 0;
theta_desired = 0;
delta_theta = 0;
W_target = 0;%(actual_target_states(wp+1,3)-actual_target_states(wp,3))/delta_t;

unwrappedCount1 = 0;
unwrappedCount2 = 0;

total_distance = 0;
% workspace variables
        UAV_path.x = r_UAV(1); UAV_path.y = r_UAV(2); UAV_path.psi = psi_UAV; UAV_path.vel = V_UAV;
        UAV_path.w = W_UAV; UAV_path.CumulativeCost = 0; UAV_path.cost = 0; UAV_path.totalDistance = 0;
        
        UAV_location = [0 r_UAV];
        V_target_actual = [0 V_tar1];
        psi_target_actual = [0 finalPath(1).psi];
        Target_location= [0 finalPath(1).x finalPath(1).y];
        Target_nextLocation = [0 finalPath(2).x finalPath(2).y];

        w_comm = [0 w_comm];
      

        speed_comm = [0 V_comm];
        
        UAV_angle_actual = [0 psi_UAV];
 
        UAV_speed_actual = [0 V_UAV];
        
        UAV_acceleration = [0 0];
        
        %%




% Constrains
a_UAV_max = 0.1;
a_UAV_min = - a_UAV_max;
psiDDot_UAV_max = 5*pi/180;
psiDot_UAV_max = 20*pi/180;
V_min = 0.25*V_UAV;
V_max = 1.25*V_UAV;


% Control Gains
% K_proximity = 0.0005;
% Kp_speed = 8;
% Kd_speed = 3;
% Kp_angle = 1;
% Kd_angle = 2;
% N = 4;

% w = 10;
% eta = 10;
% K1 = 2*eta*w; % eta is damping term, w is natural freq
% K2 = (w^2 - W_target^2)/V_tar1;
% K3 = 2*eta*w;
Kv = 1.6;
Kw = 1.2;
r_proximity = 0.5*V_tar1*delta_t;

target_proximity = distThresh;

actual_target_states = [finalPath.x; finalPath.y; finalPath.psi; finalPath.vel; finalPath.ax; finalPath.ay]';
target_final_states = actual_target_states(end,:); % at t_final


t_final = (total_distance_target/V_tar1);
t_simulation = t_final+1000;

% for t = t_final:delta_t:t_simulation
%     actual_target_states(end+1,:) = target_final_states;
%     
% end

x_target = [actual_target_states(:,1)];
y_target = [actual_target_states(:,2)];
xDot_target = [actual_target_states(:,4).*cos(actual_target_states(:,3))];
yDot_target = [actual_target_states(:,4).*sin(actual_target_states(:,3))];

time = 0;
n = 4 ;
wp = 1;
index = 1;
% Simulation solver loop
% for t = 0:delta_t:t_simulation
  %% for wp = 1:length(actual_target_states)-2

% Target States
r_target_actual = [x_target(wp) y_target(wp)];
r_target_next = [x_target(wp+1) y_target(wp+1)];

xDot_target_actual = xDot_target(wp);
yDot_target_actual = yDot_target(wp);

% psi_ref = atan2(yDot_target_actual,xDot_target_actual);

psi_target = actual_target_states(wp,3);
V_target = actual_target_states(wp,4);

count = 0;

% r_target_intermediate = [linspace(r_target_actual(1),r_target_next(1),2)' linspace(r_target_actual(2),r_target_next(2),2)']; 
 r_target_intermediate = [r_target_actual(1) r_target_actual(2) ; r_target_next(1)  r_target_next(2)]; 
    
   while(1 == 1)
% UAV MODEL 
% [LOS_rotation_rate,LOS_angle] = LOS_angle_rate(r_target_intermediate(jj,:),r_UAV,time,delta_t/n);





segmentLength = sqrt( (x_target(wp+1) - x_target(wp))^2 + (y_target(wp+1) - y_target(wp))^2);


lookAheadDist = 4.0*V_UAV*delta_t/n;

x_est_next = r_UAV(1) + V_UAV*cos(psi_UAV)*delta_t/n;
y_est_next = r_UAV(2) + V_UAV*sin(psi_UAV)*delta_t/n;

predictedFutureLocation_UAV = [x_est_next, y_est_next];
% plot(predictedFutureLocation_UAV(1)/cf_nmi_to_m,predictedFutureLocation_UAV(2)/cf_nmi_to_m,'*');    

A = [x_est_next - x_target(wp), y_est_next - y_target(wp)];
B = [ x_target(wp+1) - x_target(wp),  y_target(wp+1) - y_target(wp)];

A_cos_theta = (A(1)*B(1) + A(2)*B(2))/sqrt( B(1)^2 + B(2)^2);

m = B(2)/B(1); a = x_target(wp); b = y_target(wp); % y - y1 = m*(x-x1)

A_length = sqrt(A(1)^2 + A(2)^2);

delta_theta_prev = acos(A_cos_theta/A_length);


% if ((abs(A_cos_theta) > segmentLength )||(A_length > 2*segmentLength))




% % if (A_cos_theta < 0 )
% % %     for jj = wp:length(actual_target_states)-1
% % %         A_new = [x_est_next - x_target(jj), y_est_next - y_target(jj)];
% % %         B_new = [x_target(jj+1) - x_target(jj), y_target(jj+1) - y_target(jj)];
% % %         
% % %         A_cos_theta = (A_new(1)*B_new(1) + A_new(2)*B_new(2))/sqrt( B_new(1)^2 + B_new(2)^2);
% % %         
% % %         if (A_cos_theta > 0)
% % %             wp = jj;
% % %             break;
% % %         end
% % %             
% % %     end
% % 
% % 
% % %     if (A_length > segmentLength )
% % %         wp = wp +1;
% % %     end
% % 
% % count = count + 1;
% % 
% % % if count > 3
% % %     wp = wp + 1;
% % % end
% % 
% % A_length = sqrt(A(1)^2 + A(2)^2);
% % 
% % delta_theta_current = acos(A_cos_theta/A_length);
% % 
% % %     if (delta_theta_current - delta_theta_prev > 0)
% % %         count = count + 1;
% % %           x_lookAhead = x_target(wp+count);
% % %         y_lookAhead = y_target(wp+count);
% % %     else
% % % 
% % %         x_lookAhead = x_target(wp+1);
% % %         y_lookAhead = y_target(wp+1);
% % %     end
% %     if (wp+count < length(actual_target_states))
% %         x_lookAhead = x_target(wp+count);
% %         y_lookAhead = y_target(wp+count);
% %     else
% %         x_lookAhead = x_target(length(actual_target_states));
% %         y_lookAhead = y_target(length(actual_target_states));
% %     end
% % 
% % else % inside the circle
theta_path_segment = atan2(y_target(wp+1) - y_target(wp) , x_target(wp+1) - x_target(wp));

x_lookAhead = x_target(wp) + (A_cos_theta + lookAheadDist)*cos(theta_path_segment);
y_lookAhead = y_target(wp) + (A_cos_theta + lookAheadDist)*sin(theta_path_segment);

    if ( A_cos_theta + lookAheadDist > segmentLength )
        
            if ( wp < length(actual_target_states)-1)
                theta_path_segment_next = atan2(y_target(wp+2) - y_target(wp+1) , x_target(wp+2) - x_target(wp+1));
                x_lookAhead = x_target(wp+1) + (A_cos_theta + lookAheadDist - segmentLength)*cos(theta_path_segment_next);
                y_lookAhead = y_target(wp+1) + (A_cos_theta + lookAheadDist - segmentLength)*sin(theta_path_segment_next);
            else
                x_lookAhead = x_target(wp+1) ;
                y_lookAhead = y_target(wp+1) ;
            end     
        
    end

        
    
% end

if ( y_lookAhead ~= m*(x_lookAhead-a) + b )
    
    if ( wp < length(actual_target_states)-1)
         wp = wp + 1;
         count = 0;
         A = [x_est_next - x_target(wp), y_est_next - y_target(wp)];
         B = [ x_target(wp+1) - x_target(wp),  y_target(wp+1) - y_target(wp)];

         A_cos_theta = (A(1)*B(1) + A(2)*B(2))/sqrt( B(1)^2 + B(2)^2);
         segmentLength = sqrt( (x_target(wp+1) - x_target(wp))^2 + (y_target(wp+1) - y_target(wp))^2);
    end
        
end
        
% plot(x_lookAhead/cf_nmi_to_m,y_lookAhead/cf_nmi_to_m,'o');
    
theta_desired_prev = theta_desired;
theta_desired = atan2(y_lookAhead - r_UAV(2),x_lookAhead - r_UAV(1));

% if (theta_desired_prev - theta_desired) > pi
%     unwrappedCount1 = unwrappedCount1 + 1;
% elseif (theta_desired_prev - theta_desired) < -pi
%     unwrappedCount2 = unwrappedCount2 - 1;
% end
% theta_desired = theta_desired - 2*pi*(unwrappedCount1 + unwrappedCount2);

 % angle between 2 vectors - 1st vector: from first element of the path segment to predicted UAV location, 2nd vector:from first element of the path segment to end of it 


% Calculate Desired Linear and Angular Velocity
%V_desired = V_target;%%*cos(psi_target - psi_UAV) + K1*(r_target_intermediate(jj,1)-r_UAV(1));
% V_desired = V_target*cos(psi_target - psi_UAV) + K1*(r_target_intermediate(jj,1)-r_UAV(1));
% V_desired = 0.005*r*cos(phi);
delta_theta_prev = delta_theta;
delta_theta = theta_desired - psi_UAV;
if (delta_theta_prev - delta_theta > pi)
    delta_theta = delta_theta + 2*pi;
elseif (delta_theta_prev - delta_theta < - pi)
    delta_theta = delta_theta - 2*pi;
end
W_desired = Kw*(delta_theta)/(delta_t/n);
% W_desired = -2*sin(phi)*cos(phi)-3*phi;
%--------------------------------------CONSTRAINS----------------------------------------------------------    
%% acceleration constrains for Vmax/Vmin
a_Vmax = (V_max - V_UAV)/(delta_t/n);
a_Vmin = (V_min - V_UAV)/(delta_t/n);

%% Acceleration Lower and Upper Bound intersection of acceleration ranges [a_Vmin a_Vmax] and [a_min a_max]
if a_Vmin >= a_UAV_min
    a_lower = a_Vmin;
else
    a_lower = a_UAV_min;
end

if a_Vmax <= a_UAV_max
    a_upper =  a_Vmax;
else
    a_upper =  a_UAV_max;
end

%% Velocity Constrain - admissable velocity range [V_lower V_upper]
V_lower = V_UAV + a_lower*delta_t/n;
V_upper = V_UAV + a_upper*delta_t/n;
%------------------------------------------CONSTRAINS------------------------------------------------------ 

% Acceleration Commands

Force_command = Kv*(sqrt( (y_lookAhead - r_UAV(2))^2 + (x_lookAhead - r_UAV(1))^2) - r_proximity);
%     %%% Check whether linear acceleration command is within the acc constrain  
    if Force_command > a_upper  % Assuming unit mass and unit inertia
       Force_command = a_upper;
       V_desired = (Force_command/Kv)+V_UAV;
    elseif Force_command < a_lower 
       Force_command = a_lower;
       V_desired = (Force_command/Kv)+V_UAV;
    else
       Vdot_comm = Force_command;
    end
    
    Vdot_comm = Force_command; % Assuming unit mass and unit inertia
    
%     V_UAV = (V_UAV-V_desired)*exp(-Kv*delta_t/n) + V_desired;  
    V_UAV = V_UAV + Vdot_comm*delta_t/n ;
    
      % COMPARING THE COMMANDED AND CONSTRAINED SPEEDS
%       speed_command=V_speed_check(a_UAV_max,V_lower,V_upper,V_UAV,V_comm,delta_t/n);
    if V_UAV > V_upper
        V_UAV = V_upper;
    elseif V_UAV < V_lower
        V_UAV = V_lower;
    end
    
    
% Torque_comm = Kw*(W_desired - W_UAV);% Assuming unit mass and unit inertia
% %     %%% Check whether angular acceleration command is within the acc constrain
%     if Torque_comm > psiDDot_UAV_max
%         Torque_comm = psiDDot_UAV_max;
%         W_desired = (Torque_comm/Kw)+W_UAV;
%     elseif Vdot_comm < -psiDDot_UAV_max
%         Torque_comm = -psiDDot_UAV_max;
%         W_desired = (Torque_comm/Kw)+W_UAV;
%     end
      
    W_UAV = (W_UAV - W_desired)*exp(-Kw*delta_t/n) + W_desired;
        
%     %%% Check whether angular velocity command is within the velocity constrain
    if W_UAV > psiDot_UAV_max
        W_UAV = psiDot_UAV_max;
    elseif W_UAV < -psiDot_UAV_max
        W_UAV = -psiDot_UAV_max;
    end
    
    

        
       
    % UAV dynamics
    
%     UAV_heading = heading_command - 2*pi*floor(heading_command/2/pi);
%     % THIS DIGITAL FILTER IS THE DISCRETIZED VERSION OF THE CONTINUOUS
%     % FILTER (1/(tau*s+1))


%       tau_psi=5;  % Time constant for continuous filter
%       z_pole_psi=exp(-delta_t/n/tau_psi);
%     UAV_heading = (psi_UAV - UAV_heading_command)*z_pole_psi + UAV_heading_command;
%     
%         tau_speed = 5;
%         z_pole_speed = exp(-delta_t/n/tau_speed);        
%     UAV_speed = (V_UAV-V_comm)*z_pole_speed+V_comm;

    UAV_LinearSpeed = V_UAV;
    UAV_AngularSpeed = W_UAV;
    
    psi_comm = psi_UAV + UAV_AngularSpeed*delta_t/n;%%-2*pi*floor(UAV_heading/pi);
%     UAV_heading_command = LOS_angle;
%     tau_psi=5;  % Time constant for continuous filter
%     z_pole_psi=exp(-delta_t/n/tau_psi);
%     psi_comm = (psi_UAV - UAV_heading_command)*z_pole_psi + UAV_heading_command;
%     
        if psi_comm > pi
            psi_UAV = psi_comm - 2*pi;
        elseif psi_comm < -pi
            psi_UAV = psi_comm + 2*pi;
        else
            psi_UAV = psi_comm;
        end
    
    x_UAV = r_UAV(1) + UAV_LinearSpeed*cos(psi_UAV)*delta_t/n;
    y_UAV = r_UAV(2) + UAV_LinearSpeed*sin(psi_UAV)*delta_t/n;
    r_UAV = [x_UAV y_UAV];
    
    V_UAV = UAV_LinearSpeed;
    W_UAV = UAV_AngularSpeed;
      
    
%%% https://www.mathworks.com/help/robotics/ug/pure-pursuit-controller.html
%%% https://www.mathworks.com/help/robotics/examples/path-following-for-a-differential-drive-robot.html
    %% https://github.com/AtsushiSakai/PythonRobotics
    

        
        time = time + delta_t/n;
     
        UAV_location(end+1,:) = [time x_UAV y_UAV];
        
        V_target_actual(end+1,:) = [time V_target];
        
        psi_target_actual(end+1,:) = [time psi_target];
%         Target_location(end+1,:) = [time r_target_intermediate(jj-1,1) r_target_intermediate(jj-1,2)];

%         Target_nextLocation(end+1,:) = [time r_target_intermediate(jj,1) r_target_intermediate(jj,2)];
                
        
        w_comm(end+1,:) = [time W_UAV];      

        speed_comm(end+1,:) = [time V_comm];
        
        UAV_angle_actual(end+1,:) = [time psi_UAV];
 
        UAV_speed_actual(end+1,:) = [time V_UAV];
        
        UAV_acceleration(end+1,:) = [time Vdot_comm];
    
        [riskExposureAlongPath,PositionRisk] = RiskExposureSegment(UAV_path,PREM_value,[x_UAV y_UAV],index,mapSize(:,1), mapSize(:,2),dX,dY,PURM,X_Grid,Y_Grid,Population);
        
        
        
        UAV_path(index+1).x = x_UAV; 
        UAV_path(index+1).y = y_UAV; 
        UAV_path(index+1).psi = psi_UAV; 
        UAV_path(index+1).vel = V_UAV; 
        UAV_path(index+1).w = W_UAV;
        UAV_path(index+1).CumulativeCost = riskExposureAlongPath;
        UAV_path(index+1).cost = PositionRisk;
        
        
        x1 = [UAV_path(index).x UAV_path(index).y];
        x2 = [UAV_path(index+1).x UAV_path(index+1).y];
        
        total_distance = total_distance + Dist(x1,x2);
        UAV_path(index+1).totalDistance = total_distance; 
        
        figure(figHandle)
        
%         plot([r_target_intermediate(jj-1,1) r_target_intermediate(jj,1)]/cf_nmi_to_m , [r_target_intermediate(jj-1,2) r_target_intermediate(jj,2)]/cf_nmi_to_m , 'c-.', 'LineWidth', 2.5);
       
        if index > 1
            pause(0.01)          
            plot([UAV_location(index-1,2) UAV_location(index,2)],[UAV_location(index-1,3) UAV_location(index,3)],'c--', 'LineWidth', 4);
%             plot(predictedFutureLocation_UAV(1),predictedFutureLocation_UAV(2),'*');
%             plot(x_lookAhead,y_lookAhead,'o');
                  hold on                            
%             pause(0.1)
            
        end
          
        
    index = index +1;
    
%          if (abs(A_cos_theta) > segmentLength)%% || (A_cos_theta < 0))
% %              
%              break;
%          end
         
        
                     % Test that the UAV has reached the target's final position
    if Dist([finalPath(end).x finalPath(end).y],r_UAV) <= target_proximity
       break;
    end      
            
    end
    
    
 
RiskExposureAlongPath = UAV_path(end).CumulativeCost;    
    
   end





            
% NUMERICAL INTEGRATION to go from psi_comm_dot to psi_comm (TRAPEZOID RULE) 
   
   % PURSUIT GUIDANCE ONLY
   %psi_comm=zero_2pi(psi_comm_pre+0.1*delta_t/2*(sin(psi_ref_pre-psi_UAV_pre)+sin(psi_ref-psi_UAV)));
   % PROPORTIONAL AND PURSUIT GUIDANCE TOGETHER
  % psi_comm=zero_2pi(psi_comm_pre+K_pro*(psi_ref-psi_ref_pre)+K_pur*delta_t/2*(sin(psi_ref_pre-psi_UAV_pre)+sin(psi_ref-psi_UAV)));


        
 

    


   

