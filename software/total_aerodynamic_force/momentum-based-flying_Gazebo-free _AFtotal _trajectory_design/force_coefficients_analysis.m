%plot force coefficients

nt=size(AoA.Data,3);
tt=AoA.Time;
aoa=zeros(nt,1);
bb=zeros(nt,1);

va=zeros(3,nt);
va_norm=zeros(1,nt);

Fd=zeros(3,nt);
Fd_norm=zeros(1,nt);

Fn=zeros(3,nt);
Fn_norm=zeros(1,nt);

Cd=zeros(nt,1);
Cn=zeros(nt,1);
for i=1:nt
    aoa(i)=AoA.Data(:,:,i);
    bb(i)=beta.Data(:,:,i);
    va(:,i)=Va.Data(:,1,i);
    va_norm(i)=norm(va(:,i));
    Fd(:,i)=Fa_drag.Data(:,1,i);
    Fn(:,i)=Fa_normal.Data(:,1,i);
    Fd_norm(i)=norm(Fd(:,i));
    Fn_norm(i)=norm(Fn(:,i));
    
    %force coefficients
    Cd(i)=Fd_norm(i)/(va_norm(i)^2);
    Cn(i)=Fn_norm(i)/(va_norm(i)^2);
    
    cd_fit(i)=0.1326+0.0818*(sin(aoa(i))^2)*(cos(bb(i))^2)+0.0279*(cos(bb(i))^2);
    cn_fit(i)=0.0374*sin(2*aoa(i));
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







