close all;
clear all;
clc;

% Import dataset
load("../src/datasetTrain.mat")

% Path not the network file
onnxModelFile = "../models/model_L9_N10_100.onnx";
pytorchModelFile = "../models/model_L9_N10_100.pt";

% import network
aeroNetParam = importONNXFunction(onnxModelFile, 'aeroNet');

% initialize network
windDirection_gpu = gpuArray(windDirection_full');
jointPosDeg_gpu = gpuArray(jointPosDeg_full');

windDirection = dlarray(windDirection_gpu,'CB');
jointPosDeg  = dlarray(jointPosDeg_gpu,'CB');

windDirectionSample = windDirection(:,1);
jointPosDegSample = jointPosDeg(:,1);

predictedOutput = aeroNet(windDirectionSample, jointPosDegSample, aeroNetParam);

% save('aeroNet.mat','net');

% aeroNet = coder.loadDeepLearningNetwork('aeroNet.mat');
% 
% % % Generate code
% cfg = coder.config('lib');
% cfg.TargetLang = 'C++';
% cfg.DeepLearningConfig = coder.DeepLearningConfig(TargetLibrary='mkldnn'); 
% codegen -args {ones(1,3,'single'), ones(1,19,'single')} -config cfg aeroNet_predict

