% a trajecotry is designed for doing stability test, the trajectory is
% combined by 5 parts
%Part 1: going up, accelerate until reaches the max vel then decelerate to
%zero, time: [0,Ts] duration: Ts

%Part 2: going forward, accelerate until reaches max vel ,
%time:(Ts,Ts+Ts2/2] duration: Ts2/2

%Part 3: going forward with constant velocity which is the max vel, time:
%(Ts+Ts2/2, Ts+Ts2/2+T_const] duration: T_const

%Part 4: going forward, decelerate until reaches zero vel, time:
%(Ts+Ts2/2+T_const,Ts+Ts2/2+T_const+Ts3/2] duration: Ts3/2

%Part 5: hovering, time: (Ts+Ts2/2+T_const+Ts3/2, end] duraton: until the
%end

% the acc is defined as sin shape to avoid impact 
 % author: HUI TONG

function pos_vel_acc_jerk_CoM_des=trajectory_design(t,posCoM_int,posCoM)

pos_vel_acc_jerk_CoM_des=zeros(3,4);
 persistent posCoM_0 posCoM_1 posCoM_2 posCoM_3 posCoM_4


    if isempty(posCoM_0)
    
        posCoM_0  = posCoM_int; %initial CoM position 
    end
    
  
    
A_p1=0.8; % acc_max for going up
A_p2=3.5; % acc_max for going forward along +x axis
f_p1=0.5;%frequency of going up
f_p2=1/10;%frequency of going forward while accelerating
f_p3=1/10;%frequency of going forward while decelerating
Ts=1/f_p1;%period time for one complete circle
Ts2=1/f_p2;
Ts3=1/f_p3;
A_p3=A_p2/f_p2*f_p3; % to make sure the max velocity is the same during accelerating and decelerating
Ts_const=2;% constant velocity flying time
% desired CoM position at the end of part one
posCoM_1=posCoM_0+[0;0;A_p1/(2*pi*f_p1)*Ts-A_p1/(2*pi*f_p1)^2*sin(2*pi*f_p1*Ts)];


% desired CoM position after positive acc, accelerating
posCoM_2=posCoM_1+[A_p2/(2*pi*f_p2)*Ts2/2-A_p2/(2*pi*f_p2)^2*sin(2*pi*f_p2*Ts2/2);0;0];

%desired CoM position after constant vel
posCoM_3=posCoM_2+[A_p2/(pi*f_p2)*Ts_const;0;0];

%desired CoM position after negative acc, decelerating
posCoM_4=posCoM_3+[A_p3/(2*pi*f_p3)*(Ts3/2)+A_p3/(2*pi*f_p3)^2*sin(2*pi*f_p3*(Ts3/2));0;0];

%part one
if t<=Ts %robot goes up, a complete period 
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

%part three
elseif (Ts+Ts2/2)<t&&t<=(Ts+Ts2/2+Ts_const) % zero acc, constant v
pos_des=posCoM_2+[A_p2/(2*pi*f_p2)*2*(t-Ts-Ts2/2);0;0];
vel_des=zeros(3,1)+[A_p2/(2*pi*f_p2)*2;0;0];
acc_des=zeros(3,1);
jerk_des=zeros(3,1);
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];

%part four
elseif (Ts+Ts2/2+Ts_const)<t&&t<=(Ts+Ts2/2+Ts_const+Ts3/2) % negative acc, velocity decreases
pos_des=posCoM_3+[A_p3/(2*pi*f_p3)*(t-Ts-Ts2/2-Ts_const)+A_p3/(2*pi*f_p3)^2*sin(2*pi*f_p3*(t-Ts-Ts2/2-Ts_const));0;0];
vel_des=zeros(3,1)+[A_p3/(2*pi*f_p3)+A_p3/(2*pi*f_p3)*cos(2*pi*f_p3*(t-Ts-Ts2/2-Ts_const));0;0];
acc_des=zeros(3,1)-[A_p3*sin(2*pi*f_p3*(t-Ts-Ts2/2-Ts_const));0;0];
jerk_des=zeros(3,1)-[A_p3*(2*pi*f_p3)*cos(2*pi*f_p3*(t-Ts-Ts2/2-Ts_const));0;0];
pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];   

%part five: hovering
elseif (Ts+Ts2/2+Ts_const+Ts3/2)<t
    pos_des=posCoM_4;
    vel_des=zeros(3,1);
    acc_des=zeros(3,1);
    jerk_des=zeros(3,1);
    pos_vel_acc_jerk_CoM_des=[pos_des vel_des acc_des jerk_des];   

end
end
    

