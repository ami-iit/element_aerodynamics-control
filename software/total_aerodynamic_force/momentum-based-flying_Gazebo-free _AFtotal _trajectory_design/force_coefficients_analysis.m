%plot force coefficients and force,etc
%author: Hui Tong
close all

nt=size(AoA.Data,3); % size of time series
tt=AoA.Time; % time series

%% set size and initialize the data
aoa=zeros(nt,1);% alpha angle
bb=zeros(nt,1);% beta angle

va=zeros(3,nt);%relative velocity
va_norm=zeros(1,nt);%relative velocity norm

Fd=zeros(3,nt); %drag force
Fd_norm=zeros(1,nt);%drag force norm 

Fn=zeros(3,nt);%normal force
Fn_norm=zeros(1,nt);%normal force norm


angle1=zeros(nt,1);% angle betweem Fd (Y axis) and Va which should always be 180 deg if the implementation is correct
angle2=zeros(nt,1);% angle betweem Fn (Y axis) and Va which should always be 90 deg if the implementation is correct
Cd=zeros(nt,1);%drag coefficients
Cn=zeros(nt,1);%normal force coefficients

%% extract data from time series in workspace
for i=1:nt
    aoa(i)=AoA.Data(:,:,i);
    bb(i)=beta.Data(:,:,i);
    va(:,i)=Va.Data(:,1,i);
    va_norm(i)=norm(va(:,i));
    Fd(:,i)=Fa_drag.Data(:,1,i);
    Fn(:,i)=Fa_normal.Data(:,1,i);
    Fd_norm(i)=norm(Fd(:,i));
    Fn_norm(i)=norm(Fn(:,i));
    
    angle1(i)=atan2(norm(cross(va(:,i),Fd(:,i))),dot(va(:,i),Fd(:,i)));% \in [0,180]
    angle2(i)=atan2(norm(cross(va(:,i),Fn(:,i))),dot(va(:,i),Fn(:,i)));% \in [0,180]
    %force coefficients with only positive sign
    Cd(i)=Fd_norm(i)/(va_norm(i)^2); % fd/v^2
    Cn(i)=Fn_norm(i)/(va_norm(i)^2); % fn/v^2
    
    cd_fit(i)=0.1326+0.0818*(sin(aoa(i))^2)*(cos(bb(i))^2)+0.0279*(cos(bb(i))^2); %check the implementation of force coefficient model
    cn_fit(i)=0.0376*sin(2*aoa(i));
    
end

%% 
figure
subplot(2,1,1)
plot(tt,Fd_norm)
title('drag force')
xlabel('time')
ylabel('force N')

subplot(2,1,2)
plot(tt,Fn_norm)
title('normal force')
xlabel('time')
ylabel('force N')

figure
subplot(2,1,1)
plot(tt,Cd)
title('drag force coefficient')
xlabel('time')
ylabel('force coefficient')

hold on
plot(tt,cd_fit)

subplot(2,1,2)
plot(tt,abs(Cn))
title('normal force coefficient')
xlabel('time')
ylabel('force coefficient')
hold on
plot(tt,abs(cn_fit))
%%if the implementation of force model is correct, c_fit should be the same
%%as f/v^2

figure
subplot(2,1,1)
plot(tt,rad2deg(aoa))
title('alpha angle')
xlabel('time')
ylabel('angle degree')

subplot(2,1,2)
plot(tt,rad2deg(bb))
title('beta angle')
xlabel('time')
ylabel('angle degree')

figure
subplot(2,1,1)
plot(tt,rad2deg(angle1))
xlabel('time')
ylabel('angle degree')
title('angle between Va and Fd')

subplot(2,1,2)
plot(tt,rad2deg(angle2))
xlabel('time')
ylabel('angle degree')
title('angle between Va and Fn')


