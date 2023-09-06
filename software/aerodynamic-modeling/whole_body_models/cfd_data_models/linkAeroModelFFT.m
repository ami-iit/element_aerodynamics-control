% Author: Antonello Paolino
%
% August 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clc;

%% Initialization

% Data for the models
dataPath = './data/';
dataFile = [dataPath,'outputParameters.mat'];
load(dataFile);
jointConfigNames = fieldnames(data);

%% Aerodynamic forces application points definitions
aeroFrameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                  'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                  'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

cfdLinkNames   = {'head', 'torso', 'left_back_turbine', 'right_back_turbine', ...
                  'left_arm','left_arm_turbine','right_arm','right_arm_turbine',...
                  'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower'};

load('./src/aeroFrameTransforms.mat');

%% Load dataset
load([dataPath,'dataset.mat']);

%% Assign link data
linkIndex = 1;
cfdLinkName = cfdLinkNames{linkIndex};
aeroFrameName = aeroFrameNames{linkIndex};

% linkAoAs_full = [linkAoAs_matrix(:,linkIndex); linkAoAs_matrix(:,linkIndex+2)];
% linkSsAs_full = [linkSsAs_matrix(:,linkIndex); linkSsAs_matrix(:,linkIndex+2)];
% linkCdAs_full = [linkCdAs_matrix(:,linkIndex); linkCdAs_matrix(:,linkIndex+2)];

linkAoAs_full = linkAoAs_matrix(:,linkIndex);
linkSsAs_full = linkSsAs_matrix(:,linkIndex);
linkCdAs_full = linkCdAs_matrix(:,linkIndex);

linkClAs_full = linkClAs_matrix(:,linkIndex);
linkCsAs_full = linkCsAs_matrix(:,linkIndex);
linkCnAs_full = linkCnAs_matrix(:,linkIndex);
linkCfAs_full = linkCfAs_matrix(:,linkIndex);

%% Generate single link CdA aerodynamic model

saveFolderName = 'C:\Users\apaolino\OneDrive - Fondazione Istituto Italiano Tecnologia\Desktop\Temp\aero_models\FFT\';


[ alpha_ord , indices_ord ] = sort(linkAoAs_full);
CdA_ord = linkCdAs_full(indices_ord);

fs = 100;
[ CdA_ord , alpha_ord ] = resample(CdA_ord,alpha_ord,fs,3,1);

% plot(alpha_ord,CdA_ord);

delta_alpha  = alpha_ord(2)-alpha_ord(1);
alpha_length = alpha_ord(end)-alpha_ord(1);
Fs = alpha_length/delta_alpha;

Y = fft(CdA_ord);
n = length(alpha_ord);
f = Fs*(0:n-1)/n;

% frequency plot
fig = figure(1);
plot(f,abs(Y),'k-','LineWidth',1);
xlabel('$f$','Interpreter','latex');
ylabel('$Y_{C_D A}(f)$','Interpreter','latex');
title(cfdLinkName,'Interpreter','none');
grid on;

% saveas(fig,[saveFolderName,cfdLinkName,'_CdA_freq.svg']);

% %%%%%%%%%% selecting cutoff threshold %%%%%%%%%%%%%%%%%

% cutoff_value1 = max(abs(Y1)) * 0.06;
% Y1(abs(Y1)<cutoff_value1) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for frequency_number = [1,2,3,4,5,6,7]
    % %%%%%%%%%% selecting number of frequencies %%%%%%%%%%%%
    
    Y1 = Y;
    Y1(round(length(Y1)/2):end)=0;


    if frequency_number ~= 1
        N_freq = frequency_number;
        for i = 1 : N_freq
            [~,in_max(i)] = max(abs(Y1));
            Y1_max(i)     = Y1(in_max(i));
            main_freq(i)  = f(in_max(i));
            Y1(in_max(i)) = 0;
        end
        Y1(:) = 0;
        Y1(in_max) = Y1_max;
    else
        N_freq = length(f);
        main_freq = nan;
    end
    
    cutoff_value = min(abs(Y1(abs(Y1)>0)))/max(abs(Y1));
    disp(['for ', num2str(N_freq), 'frequencies:'])
    disp(['- the main frequencies are [', num2str(main_freq),']'])
    disp(['- the cut-off values is ', num2str(cutoff_value),'%'])
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    X1 = ifft(Y1);
    

    % plot link CdAs vs AoA
    fig = figure(frequency_number+1);
    scatter(linkAoAs_full,linkCdAs_full,[],linkSsAs_full); hold on;
    plot(alpha_ord,X1,'k-','LineWidth',2);
    xlabel('$\alpha_{link}$','Interpreter','latex');
    ylabel('$C_D A$','Interpreter','latex');
    title([cfdLinkName,' (n=', num2str(N_freq),')'],'Interpreter','none');
    grid on;
    c = colorbar;
    c.Limits = [0 180];
    c.Label.Interpreter = 'latex';
    c.Label.String = '$\beta_{link}$';
    c.Label.Position = [3, 95, 0];
    c.Label.Rotation = 0;
    c.Label.FontSize = 12;
    
%     saveas(fig,[saveFolderName,cfdLinkName,'_CdA_n',num2str(N_freq),'.svg']);

end
