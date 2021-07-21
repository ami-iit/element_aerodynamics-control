% this script is to plot the errors of CoM position, errors of
% linear momentum, error norm of CoM position, robot path, robot
% trajectory, etc and save plots after running the simulation for the use
% of stability analysis
% @author: HUI TONG


%%run this script after finishing the simulation or loading saved data

close all
close all



%load the simulink model
% MODEL_NAME = 'momentumBasedFlying.mdl';
% 
% open_system(MODEL_NAME,'loadonly');
% 
% %run simulation
% set_param(bdroot,'simulationcommand','start')

%load('/home/.../test.mat') % load the saved data in workspace 

ntime=size(linMom_err_SCOPE.time,1); % time series size
T_end=Config.simulationTime;%end time

tt=linspace(0,T_end,ntime);% for plotting
deltaT=T_end/ntime;

posCoM_err=zeros(ntime,3);
linMom_err=zeros(ntime,3);

linMom_err_norm=zeros(ntime,1);
posCoM_err_norm=zeros(ntime,1);

posCoM=zeros(ntime,3);
posCoM_des=zeros(ntime,3);

%angular momentum and rotation  attitude control
angMom_err=zeros(ntime,3);
angMom_err_norm=zeros(ntime,1);

baseRot_err=zeros(ntime,3);
baseRot_err_norm=zeros(ntime,1);

LDot_linear=zeros(ntime,3);
LDot_angular=zeros(ntime,3);

for i=1:ntime
    
    for j=1:3
    posCoM_err(i,j)=posCoM_err_SCOPE.signals(j).values(1,1,i);
    linMom_err(i,j)=linMom_err_SCOPE.signals(j).values(1,1,i);
    posCoM(i,j)=posCoM_SCOPE.signals(j).values(1,1,i);
    posCoM_des(i,j)=posCOM_des_SCOPE.signals(j).values(1,1,i);
    
    angMom_err(i,j)=angMom_err_SCOPE.signals(j).values(1,1,i);
    baseRot_err(i,j)=baseRot_err_SCOPE.signals(j).values(i);
    
    LDot_linear(i,j)=LDotEstLinear_SCOPE.signals(j).values(1,1,i);
    
    LDot_angular(i,j)=LDotEstAngular_SCOPE.signals(j).values(1,1,i);
    end
    linMom_err_norm(i)=norm(linMom_err(i,:));
    angMom_err_norm(i)=norm(angMom_err(i,:));
     baseRot_err_norm(i)=baseRotErrNorm_SCOPE.signals.values(i);
     
end

for ii=1:ntime
    posCoM_err_norm(ii)=norm(posCoM_err(ii,:));
   
    
end
posCoM_err_norm_mean=mean(posCoM_err_norm); % mean norm error


%find the maximum CoM position error 
CoM_err_max_x=max(abs(posCoM_err(:,1)));
CoM_err_max_y=max(abs(posCoM_err(:,2)));
CoM_err_max_z=max(abs(posCoM_err(:,3)));
CoM_err_max=max([CoM_err_max_x,CoM_err_max_y,CoM_err_max_z]);

%find the maximum linear momentum error
linMom_err_max_x=max(abs(linMom_err(:,1)));
linMom_err_max_y=max(abs(linMom_err(:,2)));
linMom_err_max_z=max(abs(linMom_err(:,3)));
linMom_err_max=max([linMom_err_max_x,linMom_err_max_y,linMom_err_max_z]);

    
figure(1)
plot(tt,posCoM_err,tt,CoM_err_max*ones(ntime,1))
legend('CoMerr_x','CoMerr_y','CoMerr_z')
title('posCoM error ');
xlabel('time')
ylabel('m')
figure(2)
plot(tt,linMom_err,tt,linMom_err_max*ones(ntime,1))
legend('linMomErr_x','linMomErr_y','linMomErr_z')
title('linear momentum error');
xlabel('time')
ylabel('kg x m/s')

figure(3)
plot(tt,posCoM_err_norm,'b',tt,posCoM_err_norm_mean*ones(ntime,1),'r')
legend('norm error of CoM position','mean value of norm error')
title('posCoM error norm')
xlabel('time')
ylabel('m')

figure(4)
plot(tt,baseRot_err)
legend('roll','pitch','yaw')
title('base rotation error');
xlabel('time')
ylabel('degree')

figure(5)
plot(tt,baseRot_err_norm)
title('base rotation error norm')
xlabel('time')
ylabel('degree')

figure(6)
plot(tt,angMom_err)
legend('x','y','z')
title('angular momentum error')
xlabel('time')
ylabel('kg x m/s')




%%plotting designed trajectory and robot real trajectory of CoM position 
figure(7)
plot(tt,posCoM,'-',tt,posCoM_des,'--')
legend('CoM pos x','CoM pos y','CoM pos z','desired CoM pos x','desired CoM pos y','desired CoM pos z')
xlabel('time')
ylabel('position: m')

title('robot CoM position')

%vel acc jerk z direction
figure(8)
vel_z=diff(posCoM(:,3))/deltaT;
acc_z=diff(vel_z)/deltaT;
jerk_z=diff(acc_z)/deltaT;
subplot(3,1,1)
plot(tt(1:end-1),vel_z,'b')
title('vel_z');
xlabel('time')
ylabel('m/s')
subplot(3,1,2)
plot(tt(1:end-2),acc_z,'g')
title('accCoM_z')
xlabel('time')
ylabel('m/s^2')
subplot(3,1,3)
plot(tt(1:end-3),jerk_z,'r')
title('jerkCoM_z')
xlabel('time')
ylabel('m/s^3')
%vel acc jerk y direction
figure(9)

vel_y=diff(posCoM(:,2))/deltaT;
acc_y=diff(vel_y)/deltaT;
jerk_y=diff(acc_y)/deltaT;
subplot(3,1,1)
plot(tt(1:end-1),vel_y,'b')
title('vel_y');
xlabel('time')
ylabel('m/s')

subplot(3,1,2)
plot(tt(1:end-2),acc_y,'g')
title('accCoM_y')
xlabel('time')
ylabel('m/s^2')

subplot(3,1,3)
plot(tt(1:end-3),jerk_y,'r')
title('jerkCoM_y')
xlabel('time')
ylabel('m/s^3')
% vel acc jerk x direction
figure(10)
vel_x=diff(posCoM(:,1))/deltaT;
acc_x=diff(vel_x)/deltaT;
jerk_x=diff(acc_x)/deltaT;
subplot(3,1,1)
plot(tt(1:end-1),vel_x,'b')
title('vel_x');
xlabel('time')
ylabel('m/s')

subplot(3,1,2)
plot(tt(1:end-2),acc_x,'g')
title('accCoM_x')
xlabel('time')
ylabel('m/s^2')

subplot(3,1,3)
plot(tt(1:end-3),jerk_x,'r')
title('jerkCoM_x')
xlabel('time')
ylabel('m/s^3')


% CoM velocity norm
vcom=[vel_x vel_y vel_z]';
vcom_norm=zeros(ntime,1);
for i=1:ntime-1
vcom_norm(i+1)=norm(vcom(:,i));
end

%figure()

%plot 3D trajectory
% plot3(posCoM_des(:,1),posCoM_des(:,2),posCoM_des(:,3),'--','Color','r')
% hold on   
% for i=1:ntime
%     
%     plot3(posCoM(i,1),posCoM(i,2),posCoM(i,3),'o','Color','b')
%     drawnow
%     pause
% end
% A_p=0.8; % v_max
% f_p=0.25;%T=4 s
% t=0:0.01:4;
% 
% vel_des=A_p/(2*pi*f_p)-A_p/(2*pi*f_p)*cos(2*pi*f_p*t);
% acc_des=A_p*sin(2*pi*f_p*t);
% jerk_des=-A_p*(2*pi*f_p)*cos(2*pi*f_p*t);
% plot(t,vel_des,t,acc_des,t,jerk_des)
% legend('vel','acc','jerk')

 
 %display angle of attack and Va: relative velocity
 aoa=zeros(1,ntime);
 va_norm=zeros(1,ntime);
 
 Fd_norm=zeros(1,ntime);%drag force norm 

 
 Fn_norm=zeros(1,ntime);%normal force norm
 Fa_norm=zeros(1,ntime);
 
 V_wind=v_wind.Data';
 Designed_gain=vary_gain.Data';
 Smooth_gain=smooth_gain.Data';
 for i=1:ntime
     
         aoa(i)=AoA.Data(1,1,i);
         va_norm(i)=norm(Va.Data(:,1,i));
         Fd_norm(i)=norm(Fa_drag.Data(:,1,i));
         Fn_norm(i)=norm(Fa_normal.Data(:,1,i));
         Fa_norm(i)=norm(Fa.Data(:,1,i));
         
 end
 
 figure(11)
 plot(tt,aoa)
 title('alpha angle');
 xlabel('time')
 ylabel('degree')

figure(12)
plot(tt,va_norm)
title('relative velocity');
xlabel('time')
ylabel('m/s')

figure(13)
subplot(3,1,1)
plot(tt,Fa_norm)
title('total aerodynamic force')
xlabel('time')
ylabel('N')


subplot(3,1,2)
plot(tt,Fd_norm)
title('drag force')
xlabel('time')
ylabel('N')

subplot(3,1,3)
plot(tt,Fn_norm)
title('normal force')
xlabel('time')
ylabel('N')

figure(14)
plot(tt,LDot_linear)
legend('x','y','z')
title('linear momentum derivative')
xlabel('time')
ylabel('N')

figure(15)
plot(tt,LDot_angular)
legend('x','y','z')
title('angular momentum derivative')
xlabel('time')
ylabel('N')


figure(16)
plot(tt,V_wind)
legend('x','y','z')
title('wind velocity')
xlabel('time')
ylabel('m/s')

figure(17)
plot(tt,Designed_gain)
legend('x','y','z')
title('Designed gain')
xlabel('time')
ylabel('scalar value')

figure(18)
plot(tt,Smooth_gain)
legend('x','y','z')
title('Smooth gain')
xlabel('time')
ylabel('scalar value')

figure(19)
plot(tt,linMom_err_norm)
title('linear momentum error norm')
xlabel('time')
ylabel('kg x m/s')

figure(20)
plot(tt,angMom_err_norm)
title('angular momentum error norm')
xlabel('time')
ylabel('kg x m/s')

figure(21)
plot(tt(1:end-1),vcom)
title('CoM velocity')
legend('x','y','z')
xlabel('time')
ylabel('m/s')

figure(22)
plot(tt,vcom_norm)
title('CoM velocity norm')
xlabel('time')
ylabel('m/s')






%save figures .tiff
path='/home/...';    
saveas(figure(1),fullfile(path,['pos_err' '.tiff']));
saveas(figure(2),fullfile(path,['lin_mom_err' '.tiff']));
saveas(figure(3),fullfile(path,['pos_err_norm' '.tiff']));
saveas(figure(4),fullfile(path,['base_rot_err' '.tiff']));
saveas(figure(5),fullfile(path,['base_rot_err_norm' '.tiff']));
saveas(figure(6),fullfile(path,['angu_mom_err' '.tiff']));
saveas(figure(7),fullfile(path,['robot_trajectory' '.tiff']));
saveas(figure(8),fullfile(path,['velz' '.tiff']));
saveas(figure(9),fullfile(path,['vely' '.tiff']));
saveas(figure(10),fullfile(path,['velx' '.tiff']));
saveas(figure(11),fullfile(path,['aoa' '.tiff']));
saveas(figure(12),fullfile(path,['va_norm' '.tiff']));
saveas(figure(13),fullfile(path,['aero_force' '.tiff']));
saveas(figure(14),fullfile(path,['LDot_linear' '.tiff']));
saveas(figure(15),fullfile(path,['LDot_angular' '.tiff']));
saveas(figure(16),fullfile(path,['wind_velocity' '.tiff']));
saveas(figure(17),fullfile(path,['designed_gain' '.tiff']));
saveas(figure(18),fullfile(path,['smooth_gain' '.tiff']));
saveas(figure(19),fullfile(path,['linMom_err_norm' '.tiff']));
saveas(figure(20),fullfile(path,['angMom_err_norm' '.tiff']));
saveas(figure(21),fullfile(path,['vcom' '.tiff']));
saveas(figure(22),fullfile(path,['vcom_norm' '.tiff']));


save(fullfile(path,'test.mat'));    