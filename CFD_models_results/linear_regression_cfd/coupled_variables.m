clc
clear all
close all

%this script is to load all the cfd data and proceed linear regression
%with the proposed models
% author: HUI TONG


%% LOAD DATA
%% data from sheet "Beta=180" 
data1=readmatrix("CFD_Data_Collection.xlsx",'Sheet','Beta=180','Range','E16:AC16');
A1=[data1(1);data1(6);data1(11);data1(16);data1(21)];%deg  alpha angle
B1=180+zeros(5,1); %deg beta angle
Cd1=[data1(4);data1(9);data1(14);data1(19);data1(24)]; % drag coefficients fd/v^2
Cn1=[data1(5);data1(10);data1(15);data1(20);data1(25)]; % normal force coefficients fl/v^2
ns1=size(A1,1);%data size
%% data from sheet "Beta=270" deg
data2=readmatrix("CFD_Data_Collection.xlsx",'Sheet','Beta=270','Range','E16:AC16');
A2=[data2(1);data2(6);data2(11);data2(16);data2(21)];%deg
B2=3*90*ones(5,1); %deg
Cd2=[data2(4);data2(9);data2(14);data2(19);data2(24)];
Cn2=[data2(5);data2(10);data2(15);data2(20);data2(25)];
ns2=size(A2,1);
%% data from sheet "Alpha=90" deg
data3=readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=90','Range','E16:AC16');
B3=[data3(1);data3(6);data3(11);data3(16);data3(21)];%deg beta in cfd
ns3=size(B3,1);% size of expanded data

Cd3=[data3(4);data3(9);data3(14);data3(19);data3(24)];
Cn3=[data3(5);data3(10);data3(15);data3(20);data3(25)];

%expand the data from (0,pi) into domain (0,2pi)
% B3=[B3;180+B3(2:end)];
% Cd3=[Cd3;flip(Cd3(1:end-1))];
% Cl3=[Cl3;flip(Cl3(1:end-1))];

A3=90*ones(ns3,1); %deg alpha
%% data from sheet "Alpha=60" deg
data4=readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=60','Range','E16:AC16');
B4=[data4(1);data4(6);data4(11);data4(16);data4(21)];%deg
ns4=size(B4,1);% size of expanded data

Cd4=[data4(4);data4(9);data4(14);data4(19);data4(24)];
Cn4=[data4(5);data4(10);data4(15);data4(20);data4(25)];
%expand the data into domain 2pi
% B4=[B4;180+B4(2:end)];
% Cd4=[Cd4;flip(Cd4(1:end-1))];
% Cl4=[Cl4;flip(Cl4(1:end-1))];
A4=60*ones(ns4,1); %deg
%% data from sheet "Alpha=120" deg
data5=readmatrix("CFD_Data_Collection.xlsx",'Sheet','Alpha=120','Range','E16:AC16');
B5=[data5(1);data5(6);data5(11);data5(16);data5(21)];%deg
ns5=size(B5,1);% size of expanded data

Cd5=[data5(4);data5(9);data5(14);data5(19);data5(24)];
Cn5=[data5(5);data5(10);data5(15);data5(20);data5(25)];
%expand the data into domain 2pi
% B5=[B5;180+B5(2:end)];
% Cd5=[Cd5;flip(Cd5(1:end-1))];
% Cl5=[Cl5;flip(Cl5(1:end-1))];

A5=120*ones(ns5,1); %deg

%% collect all data
A=[A1;A2;A3;A4;A5]; % alpha angle of all data
B=[B1;B2;B3;B4;B5]; % beta angle of all data 

Cd=[Cd1;Cd2;Cd3;Cd4;Cd5]; % drag coefficients of all data  fd/v^2
Cn=[Cn1;Cn2;Cn3;Cn4;Cn5]; % normal force coefficients of all data  fl/v^2
A_r=deg2rad(A); % rad
B_r=deg2rad(B); % rad
tt=linspace(0,360,100);  % for plotting estimated model
tt_r=deg2rad(tt); % rad
ns=size(A,1); % size of the data set

%% Linear Regression with different models for drag force coefficient


%% Model 01   cd=(c0 + 2*c1*sin(a)^2)*(c2+c3*cos(beta)^2)    (c0, c1) and (c2, c3) are extracted seperately

% c0 and c1 are estimated by the data in sheet 1 Beta=0 Ridge Regression is
% used due to small number of data 
A1_r=deg2rad(A1);
Y1=[Cd1;Cn1];
X1_alpha=[ones(ns1,1) 2*(sin(A1_r).^2); zeros(ns1,1) sin(2*A1_r)];

%least square  LS "\"
%C_LS=X\Y;

%ridge regression RG
lambda=0.8;
C1_RG=(transpose(X1_alpha)*X1_alpha+lambda*eye(size(X1_alpha,2)))\(transpose(X1_alpha)*Y1);

%estimated cd1 and cl1
aa=linspace(0,180,100);
aa_r=deg2rad(aa);
cd1_RG=C1_RG(1)+2*C1_RG(2)*(sin(aa_r).^2);
cl1_RG=C1_RG(2)*sin(2*aa_r);

%plot cfd data and estimated model
figure
plot(A1,Cd1,'x','MarkerSize',8)

hold on
plot(aa,cd1_RG,'r')
legend('cfd data beta=0','RG estimated cd')
xlabel('angle of attack /deg');
ylabel('drag coefficient')
title('fd/v^2')
%plot cl 
figure;
plot(A1,Cn1,'x','MarkerSize',8)

hold on
plot(aa,cl1_RG,'r')
legend('cfd data beta=0','RG estimated cl')
xlabel('angle of attack');
ylabel('lift coefficient')
title('fl/v^2')

% c2 and c3 are estimated by the data in sheet 3,4,5 Alpha=90,60,120 LS Regression is
% used due to the fact that the chosen model already has penalty from cfd data 

%calculate the mean value among Alpha=90,60,120
Cd3_am=Cd3/Cd3(1);
Cd4_am=Cd4/Cd4(1);
Cd5_am=Cd5/Cd5(1);
Cd_mean=(Cd3_am+Cd4_am+Cd5_am)/3;

B3_r=deg2rad(B3);

X1_beta=[ones(size(Cd_mean,1),1) cos(B3_r).^2 ];
C_beta=X1_beta\Cd_mean;
Cd_beta=C_beta(2)*(cos(tt_r).^2)+C_beta(1);

figure
plot(B3,Cd_mean,'b-x','MarkerSize',8);
hold on 
plot(tt,Cd_beta,'r');
xlabel('beta /deg');
ylabel('Cd/Cd_0')
title('drag coefficients')
legend('mean value from cfd','estimated')




c0_model1=C1_RG(1);
c1_model1=C1_RG(2);
c2_model1=C_beta(1);
c3_model1=C_beta(2);

C1_est=[c0_model1;c1_model1;c2_model1;c3_model1];
% calculate RMSE of Model 01 for all the data
Y_est1=zeros(ns,1);%estimated value
R1=zeros(ns,1); %error square
for i=1:ns
    
Y_est1(i)=(c0_model1+2*c1_model1*(sin(A_r(i))^2))*(c2_model1+c3_model1*(cos(B_r(i))^2));

R1(i)=(Y_est1(i)-Cd(i))^2;
end

RMSE1=sqrt(sum(R1)/ns);
%% Model 02 cd=c0 + c1*sin(a)^2 * cos(beta)^2 + c2*sin(a)^2 + c3*cos(beta)^2


X2=[ones(ns,1) (sin(A_r).^2).*(cos(B_r).^2)  sin(A_r).^2  cos(B_r).^2]; 
X2_app=[(sin(A_r).^2).*(cos(B_r).^2)  sin(A_r).^2  cos(B_r).^2]; % used for Regression Learner app

C2_LS=X2\Cd;%least square

Y_est2=zeros(ns,1);
R2=zeros(ns,1);
for i=1:ns
    
Y_est2(i)=C2_LS(1)+C2_LS(2)*(sin(A_r(i))^2)*(cos(B_r(i))^2)+C2_LS(3)*(sin(A_r(i))^2) +C2_LS(4)*(cos(B_r(i))^2);

R2(i)=(Y_est2(i)-Cd(i))^2;
end

RMSE2=sqrt(sum(R2)/ns);

%% Model 03 cd=c0 + c1*sin(a)^2 * cos(beta)^2 + c2*cos(beta)^2
X3=[ones(ns,1) (sin(A_r).^2).*(cos(B_r).^2)  cos(B_r).^2];   
X3_app=[(sin(A_r).^2).*(cos(B_r).^2)  cos(B_r).^2];
C3_LS=X3\Cd;
Y_est3=zeros(ns,1);
R3=zeros(ns,1);
for i=1:ns
    
Y_est3(i)=C3_LS(1)+C3_LS(2)*(sin(A_r(i))^2)*(cos(B_r(i))^2) +C3_LS(3)*(cos(B_r(i))^2);
R3(i)=(Y_est3(i)-Cd(i))^2;
end

RMSE3=sqrt(sum(R3)/ns);

%% Model 04 cd=c0 + c1*sin(a)^2 * cos(beta)^2
X4_app=(sin(A_r).^2).*(cos(B_r).^2);

%% Model 05 cd=c0+c1*cos(2a)+c2*cos(4a)+c3*cos(2beta)+c4cos(4beta)
X5_app=[cos(2*A_r) cos(4*A_r) cos(2*B_r) cos(4*B_r)];



%% Linear Regression with different models for normal force coefficient

%% Model01-cn cn=c_n*sin(2a)
X_cn=sin(2*A_r);
c_n=X_cn\Cn;
Y_Cn_est=c_n*sin(2*A_r);
Cn_fit=c_n*sin(2*aa_r);

R=sum((Y_Cn_est-Cn).^2)/ns;
RMSE_cn=sqrt(R);

figure
plot3(A,B,Cn,'bx','MarkerSize',8)
hold on
plot3(aa,tt,Cn_fit,'r')
xlabel('alpha angle /deg');
ylabel('Cn')
title('normal force coefficients')
legend('Cn cfd','estimated Cn')

%% Model 02-cn  cn=d1*sin(2a)*cos(beta)^2+d2*sin(2a)cos(beta)+d3*sin(2a)
X2_cn=[sin(2*A_r).*(cos(B_r).^2) sin(2*A_r).*cos(B_r) sin(2*A_r)];

c_n2=X2_cn\Cn;
Y_Cn_est2=c_n2(1)*sin(2*A_r).*(cos(B_r).^2)+c_n2(2)*sin(2*A_r).*cos(B_r)+c_n2(3)*sin(2*A_r);


R2=sum((Y_Cn_est2-Cn).^2)/ns;
RMSE_cn2=sqrt(R2);

%% Model 03-cn  cn=d1*sin(2a)*cos(beta)^2+d2*sin(2a)cos(beta) **
X3_cn=[sin(2*A_r).*(cos(B_r).^2) sin(2*A_r).*cos(B_r)];

c_n3=X3_cn\Cn;
Y_Cn_est3=c_n3(1)*sin(2*A_r).*(cos(B_r).^2)+c_n3(2)*sin(2*A_r).*cos(B_r);


R3=sum((Y_Cn_est3-Cn).^2)/ns;
RMSE_cn3=sqrt(R3);


%% Model 04 cn=c1*sin(2a)*cos(2beta)+c2*sin(2a)*cos(4beta)
x4_cn=[sin(2*A_r).*cos(2*B_r) sin(2*A_r).*cos(4*B_r)];




