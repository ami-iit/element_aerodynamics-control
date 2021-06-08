%run this script after finishing simulation 

na=length(Va.Data(1,1,:));
G=M.Data(1,1,1)*9.8;
va_equi=-sqrt(G/aerodynamics_config.Ka(3)/aerodynamics_config.C_D(3));
for i=1:na
    Vaa(i)=Va.Data(3,1,i);
    Faa(i)=Fa.Data(3,1,i);
    COM_z(i)=posCOM.Data(3,1,i);
    LinearMomentum_z(i)=Momentum.Data(i,3);
end
TT=linspace(1,1000,na);
figure(2);
plot(TT,Vaa,'b')
hold on
plot(TT,va_equi*ones(na,1),'r')
legend('Va_z','VaEquilirium_z')

figure(3)
plot(TT,Faa,'b')
hold on
plot(TT,G*ones(na,1),'g')
legend('dragForce_z','mg')

figure(4)
plot(TT,COM_z,'b')

hold on
plot(TT,LinearMomentum_z,'r')
legend("CoM_z","LinearMomentum_z");

figure(5)
diffCOM=diff(posCOM.Data(3,1,:));
diffMM=diff(Momentum.Data(:,3));
DotCOM_z=zeros(na-1,1);
DotLinearMomentum_z=zeros(na-1,1);
for i=1:na-1
DotCOM_z(i)=diffCOM(1,1,i);

DotLinearMomentum_z(i)=diffMM(i,1);
end
plot(TT(2:end),DotCOM_z)

hold on
plot(TT(2:end),DotLinearMomentum_z)