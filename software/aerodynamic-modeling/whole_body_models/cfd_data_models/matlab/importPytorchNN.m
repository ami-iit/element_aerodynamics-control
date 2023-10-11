close all;
clear all;
clc;

% Import dataset
load("../src/datasetTrain.mat")

% Path not the network file
onnxModelFile = "../src/model.onnx";

% Load the ONNX model into MATLAB with specified data formats
aeroNetParam = importONNXFunction(onnxModelFile,"./aeroNet");

pitchAngles_cpu = pitchAngles_full;
yawAngles_cpu   = yawAngles_full;
jointPosDeg_cpu = jointPosDeg_full;

pitchAngles_gpu = gpuArray(pitchAngles_full);
yawAngles_gpu = gpuArray(yawAngles_full);
jointPosDeg_gpu = gpuArray(jointPosDeg_full);

tic
for i = 1:10
predictedOutput = aeroNet(pitchAngles_cpu(1,1)', yawAngles_cpu(1,1)', jointPosDeg_cpu(1,:)', aeroNetParam);
end
toc

tic
for i = 1:10
predictedOutput = aeroNet(pitchAngles_gpu(1,1)', yawAngles_gpu(1,1)', jointPosDeg_gpu(1,:)', aeroNetParam);
end
toc

scatter(linkAoAs_matrix(:,1),predictedOutput(:,1));