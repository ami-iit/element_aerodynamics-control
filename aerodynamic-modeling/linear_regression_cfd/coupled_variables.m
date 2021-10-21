clc
clear variables
close all

%this script is to load all the cfd data and apply linear regression
%with the proposed models to obtain the constant coefficients
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

%% Model identification for drag force coefficient


%% Model  cd=c0 + c1*sin(a)^2 * cos(beta)^3 + c2*cos(beta)^3+c3*cos(beta)^2
%XD=[ones(ns,1) (sin(A_r).^2).*(cos(B_r).^2)  cos(B_r).^2];   
XD_app_use=[(sin(A_r).^2).*(cos(B_r).^3)  cos(B_r).^3 cos(B_r).^2];



%% Model identification for normal force coefficient


%% Model  cn=c4+c5*sin(2a)*cos(beta)^2
XN_app_use=sin(2*A_r).*(cos(B_r).^2);






