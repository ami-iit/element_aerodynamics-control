% a trajecotry is designed for doing stability test, the trajectory is
% combined by 4 parts: up (+z) -> front (+x) 
% at the starting and end point of each part , the CoM velocity is zero ,
% the acc is defined as sin shape to avoid impact at the beginning 
 % author: HUI TONG

function pos_vel_acc_jerk_CoM_des=trajectory_design(t,posCoM_int,posCoM)

pos_vel_acc_jerk_CoM_des=zeros(3,4);
 persistent posCoM_0 posCoM_1 posCoM_2 posCoM_3


    if isempty(posCoM_0)
    
        posCoM_0  = posCoM_int; %initial CoM position 
    end
    
  
    
A_p1=0.8; % acc_max
A_p2=4; % A_p max=6.5... 4 is still high, gain tunning required
f_p1=0.5;%frequency
f_p2=0.2;%0.25;
Ts=1/f_p1;%period time for one complete circle
Ts2=1/f_p2;
Ts3=4;% constant velocity flying time
% desired CoM position at the end of part one
posCoM_1=posCoM_0+[0;0;A_p1/(2*pi*f_p1)*Ts-A_p1/(2*pi*f_p1)^2*sin(2*pi*f_p1*Ts)];


% desired CoM position at the end of part two
posCoM_2=posCoM_1+[A_p2/(2*pi*f_p2)*Ts2/2-A_p2/(2*pi*f_p2)^2*sin(2*pi*f_p2*Ts2/2);0;0];
%smooth trajectory 
posCoM_3=posCoM_2+[A_p2/(pi*f_p2)*Ts3;0;0];
%part one
if t<=Ts %robot goes up
pos_des=posCoM_0+[0;0;A_p1/(2*pi*f_p1)*t-A_p1/(2*pi*f_p1)^2*sin(2*pi*f_p1*t)];
vel_des=zeros(3,1)+[0;0;A_p1/(2*pi*f_p1)-A_p1/(2*pi*f_p1)*cos(2*pi*f_p1*t)];
acc_des=zeros(3,1)+[0;0;A_p1*sin(2*pi*f_p1*t)];
jerk_des=zeros(3,1)+[0;0;A_p1*(2*pi*f_p1)*cos(2*pi*f_p1*t)];

pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

%part two %robot goes forward
elseif Ts<t&&t<=(Ts+Ts2/2) %positive acc, velocity increases
    
pos_des=posCoM_1+[A_p2/(2*pi*f_p2)*(t-Ts)-A_p2/(2*pi*f_p2)^2*sin(2*pi*f_p2*(t-Ts));0;0];
vel_des=zeros(3,1)+[A_p2/(2*pi*f_p2)-A_p2/(2*pi*f_p2)*cos(2*pi*f_p2*(t-Ts));0;0];
acc_des=zeros(3,1)+[A_p2*sin(2*pi*f_p2*(t-Ts));0;0];
jerk_des=zeros(3,1)+[A_p2*(2*pi*f_p2)*cos(2*pi*f_p2*(t-Ts));0;0];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

elseif (Ts+Ts2/2)<t&&t<=(Ts+Ts2/2+Ts3) % zero acc, constant v
pos_des=posCoM_2+[A_p2/(2*pi*f_p2)*2*(t-Ts-Ts2/2);0;0];
vel_des=zeros(3,1)+[A_p2/(2*pi*f_p2)*2;0;0];
acc_des=zeros(3,1);
jerk_des=zeros(3,1);
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

elseif (Ts+Ts2/2+Ts3)<t&&t<=(Ts+Ts2+Ts3) % negative acc, velocity decreases
pos_des=posCoM_3+[A_p2/(2*pi*f_p2)*(t-Ts-Ts2/2-Ts3)+A_p2/(2*pi*f_p2)^2*sin(2*pi*f_p2*(t-Ts-Ts2/2-Ts3));0;0];
vel_des=zeros(3,1)+[A_p2/(2*pi*f_p2)+A_p2/(2*pi*f_p2)*cos(2*pi*f_p2*(t-Ts-Ts2/2-Ts3));0;0];
acc_des=zeros(3,1)-[A_p2*sin(2*pi*f_p2*(t-Ts-Ts2/2-Ts3));0;0];
jerk_des=zeros(3,1)-[A_p2*(2*pi*f_p2)*cos(2*pi*f_p2*(t-Ts-Ts2/2-Ts3));0;0];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];   
end
end
    

