% this script is to run the simulation, plot the errors of CoM position, errors of
% linear momentum, error norm of CoM position, robot path, robot
% trajectory,..

%load the simulink model
MODEL_NAME = 'momentumBasedFlying.mdl';

open_system(MODEL_NAME,'loadonly');

%run simulation
set_param(bdroot,'simulationcommand','start')


% handle the data extracted from simulink

%time series
ntime=size(linMom_err_SCOPE.time,1);
T_end=Config.simulationTime;%end time

tt=linspace(0,T_end,ntime);% for plotting


posCoM_err=zeros(ntime,3);
linMom_err=zeros(ntime,3);
posCoM_err_norm=zeros(ntime,1);
posCoM=zeros(ntime,3);
posCoM_des=zeros(ntime,3);
for i=1:ntime
    
    for j=1:3
    posCoM_err(i,j)=posCoM_err_SCOPE.signals(j).values(1,1,i);
    linMom_err(i,j)=linMom_err_SCOPE.signals(j).values(1,1,i);
    posCoM(i,j)=posCoM_SCOPE.signals(j).values(1,1,i);
    posCoM_des(i,j)=posCOM_des_SCOPE.signals(j).values(1,1,i);
    
    end
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
plot(tt,posCoM_err,tt,CoM_err_max*ones(ntime,1),tt,-CoM_err_max*ones(ntime,1))
legend('CoMerr_x','CoMerr_y','CoMerr_z','CoMerr_max')
title('posCoM error');

figure(2)
plot(tt,linMom_err,tt,linMom_err_max*ones(ntime,1),tt,-linMom_err_max*ones(ntime,1))
legend('linMomErr_x','linMomErr_y','linMomErr_z','linMomErr_max')
title('linear momentum error');

figure(3)
plot(tt,posCoM_err_norm,'b',tt,posCoM_err_norm_mean*ones(ntime,1),'r')
legend('norm error of CoM position','mean value of norm error')



%%plotting designed trajectory and robot real trajectory of CoM position 
figure(4)
plot(tt,posCoM,'-',tt,posCoM_des,'--')
legend('real trajectory x','real trajectory y','real trajectory z','desired trajectory x','desired trajectory y','desired trajectory z')

title('robot trajectory')

%vel acc jerk z direction
figure(5)
vel=diff(posCoM(:,3))./diff(tt);
acc=diff(vel)./diff(tt);
jerk=diff(acc)./diff(tt);
subplot(3,1,1)
plot(tt(1:end-1),vel,'b')
title('vel_z');

subplot(3,1,2)
plot(tt(1:end-2),acc,'g')
title('accCoM_z')

subplot(3,1,3)
plot(tt(1:end-3),jerk,'r')
title('jerkCoM_z')

%vel acc jerk y direction
figure(6)

vel=diff(posCoM(:,2))./diff(tt);
acc=diff(vel)./diff(tt);
jerk=diff(acc)./diff(tt);
subplot(3,1,1)
plot(tt(1:end-1),vel,'b')
title('vel_y');

subplot(3,1,2)
plot(tt(1:end-2),acc,'g')
title('accCoM_y')

subplot(3,1,3)
plot(tt(1:end-3),jerk,'r')
title('jerkCoM_y')

% vel acc jerk x direction
figure(7)
vel=diff(posCoM(:,1))./diff(tt);
acc=diff(vel)./diff(tt);
jerk=diff(acc)./diff(tt);
subplot(3,1,1)
plot(tt(1:end-1),vel,'b')
title('vel_x');

subplot(3,1,2)
plot(tt(1:end-2),acc,'g')
title('accCoM_x')

subplot(3,1,3)
plot(tt(1:end-3),jerk,'r')
title('jerkCoM_x')

%figure(8)

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

 
 %display angle of attack and Va
 aoa=zeros(13,ntime);
 va_norm=zeros(13,ntime);
 for i=1:ntime
     for j=1:13
         aoa(j,i)=AoA.Data(1,j,i);
         va_norm(j,i)=norm(Va.Data(:,j,i));
     end
 end
 
 figure(8)
 plot(tt,aoa)
 title('angle of attack');

%display cd and cl
cd=zeros(13,ntime);
cl=zeros(13,ntime);
for  i=1:ntime
     for j=1:13
         cd(j,i)=aerodynamics_config.C_0(j)+2*aerodynamics_config.C_1(j)*sin(aoa(j,i))^2;
         cl(j,i)=aerodynamics_config.C_1(j)*sin(2*aoa(j,i));
     end
end
figure(9)
plot(tt,cd)
title('drag coefficient');

figure(10)
plot(tt,cl)
title('lift coefficient');

figure(11)
plot(tt,va_norm)
title('relative velocity');

%save figures
% path='/home/tong_hui/Documents/iit_tong/Aerodynamics_control_element/stability_test/trajectory_01_plus/random_co_c1/wind_1_1/c0_0.5/c0_0.5_c1_0.5';    
% saveas(figure(1),fullfile(path,['pos_err' '.jpg']));
% saveas(figure(3),fullfile(path,['pos_err_norm' '.jpg']));
% saveas(figure(4),fullfile(path,['robot_path' '.jpg']));
% saveas(figure(8),fullfile(path,['aoa' '.jpg']));
% saveas(figure(9),fullfile(path,['cd' '.jpg']));
% saveas(figure(10),fullfile(path,['cl' '.jpg']));
% saveas(figure(11),fullfile(path,['va_norm' '.jpg']));
%save(fullfile(path,'test.mat'));    