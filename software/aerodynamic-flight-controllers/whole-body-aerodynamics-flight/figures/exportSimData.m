close all;
clear all;
clc;

for expType = 1:5
    
    
    %% Load mat file and define paths
    
    if expType == 1
        expMatFile = '../experiments/2023-11-13/exp_17-27.mat';
        exportName = '17-27';
    elseif expType == 2
        expMatFile = '../experiments/2023-11-13/exp_17-30.mat';
        exportName = '17-30';
    elseif expType == 3
        expMatFile = '../experiments/2023-11-13/exp_17-36.mat';
        exportName = '17-36';
    elseif expType == 4
        expMatFile = '../experiments/2023-11-13/exp_17-43.mat';
        exportName = '17-43';
    elseif expType == 5
        expMatFile = '../experiments/2024-01-30/exp_12-40.mat';
        exportName = '12-40';
    end
    
    load(expMatFile);
    
    %% assign variables

    time = out.jointPosMeas_SCOPE.time;

    var = out.jointPosMeas_SCOPE.signals;
    jointPos = zeros(1, length(time));
    start_idx = 1;
    for i = 1 : length(var)
        n = size(squeeze(var(i).values));
        end_idx = start_idx + n(1) - 1;
        jointPos(start_idx:end_idx, :) = squeeze(var(i).values);
        start_idx = end_idx+1;
    end
    jointPos = transpose(jointPos);

    var = out.baseRot_SCOPE.signals;
    baseRot = zeros(1, length(time));
    start_idx = 1;
    for i = 1 : length(var)
        n = size(squeeze(var(i).values));
        end_idx = start_idx + n(2) - 1;
        baseRot(start_idx:end_idx, :) = squeeze(var(i).values);
        start_idx = end_idx+1;
    end
    baseRot = transpose(baseRot);

    var = out.basePos_SCOPE.signals;
    basePos = zeros(1, length(time));
    start_idx = 1;
    for i = 1 : length(var)
        n = size(squeeze(var(i).values));
        end_idx = start_idx + n(2) - 1;
        basePos(start_idx:end_idx, :) = squeeze(var(i).values);
        start_idx = end_idx+1;
    end
    basePos = transpose(basePos);
    
    save([exportName,'_joint-positions.mat'],'time','jointPos','baseRot','basePos');

end

