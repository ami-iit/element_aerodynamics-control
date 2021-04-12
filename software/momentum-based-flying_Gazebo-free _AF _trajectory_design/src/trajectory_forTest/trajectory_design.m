% a trajecotry is designed for doing stability test, the trajectory is
% combined by 4 parts: up (+z) -> left(+y) -> front (+x) -> down(-z) 
% at the starting and end point of each part , the CoM velocity is zero ,
% the acc is defined as sin shape to avoid impact at the beginning 


function pos_vel_acc_jerk_CoM_des=trajectory_design(t,posCoM_int,posCoM)

pos_vel_acc_jerk_CoM_des=zeros(3,4);
 persistent posCoM_0 


    if isempty(posCoM_0)
    
        posCoM_0  = posCoM_int; %initial CoM position 
    end
    
  
    
A_p=0.8; % acc_max
f_p=0.25;%frequency
Ts=1/f_p;%period time for one complete circle
% desired CoM position at the end of part one
posCoM_1=posCoM_0+[0;0;A_p/(2*pi*f_p)*Ts-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*Ts)];

% desired CoM position at the end of part two
posCoM_2=posCoM_1+[0;A_p/(2*pi*f_p)*Ts-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*Ts);0];

% desired CoM position at the end of part three
posCoM_3=posCoM_2+[A_p/(2*pi*f_p)*Ts-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*Ts);0;0];
%smooth trajectory 

%part one
if t<=4
pos_des=posCoM_0+[0;0;A_p/(2*pi*f_p)*t-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*t)];
vel_des=zeros(3,1)+[0;0;A_p/(2*pi*f_p)-A_p/(2*pi*f_p)*cos(2*pi*f_p*t)];
acc_des=zeros(3,1)+[0;0;A_p*sin(2*pi*f_p*t)];
jerk_des=zeros(3,1)-[0;0;A_p*(2*pi*f_p)*cos(2*pi*f_p*t)];

pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

%part two
elseif 4<t&&t<=8
    
pos_des=posCoM_1+[0;A_p/(2*pi*f_p)*(t-Ts)-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*(t-Ts));0];
vel_des=zeros(3,1)+[0;A_p/(2*pi*f_p)-A_p/(2*pi*f_p)*cos(2*pi*f_p*(t-Ts));0];
acc_des=zeros(3,1)+[0;A_p*sin(2*pi*f_p*(t-Ts));0];
jerk_des=zeros(3,1)-[0;A_p*(2*pi*f_p)*cos(2*pi*f_p*(t-Ts));0];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

%part three
    
elseif 8<t&&t<=12

pos_des=posCoM_2+[A_p/(2*pi*f_p)*(t-2*Ts)-A_p/(2*pi*f_p)^2*sin(2*pi*f_p*(t-2*Ts));0;0];
vel_des=zeros(3,1)+[A_p/(2*pi*f_p)-A_p/(2*pi*f_p)*cos(2*pi*f_p*(t-2*Ts));0;0];
acc_des=zeros(3,1)+[A_p*sin(2*pi*f_p*(t-2*Ts));0;0];
jerk_des=zeros(3,1)-[A_p*(2*pi*f_p)*cos(2*pi*f_p*(t-2*Ts));0;0];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

%part four
elseif 12<t&&t<=16
pos_des=posCoM_3+[0;0;-A_p/(2*pi*f_p)*(t-3*Ts)+A_p/(2*pi*f_p)^2*sin(2*pi*f_p*(t-3*Ts))];
vel_des=zeros(3,1)+[0;0;-A_p/(2*pi*f_p)+A_p/(2*pi*f_p)*cos(2*pi*f_p*(t-3*Ts))];
acc_des=zeros(3,1)+[0;0;-A_p*sin(2*pi*f_p*(t-3*Ts))];
jerk_des=zeros(3,1)-[0;0;-A_p*(2*pi*f_p)*cos(2*pi*f_p*(t-3*Ts))];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];
    
end
end
    

