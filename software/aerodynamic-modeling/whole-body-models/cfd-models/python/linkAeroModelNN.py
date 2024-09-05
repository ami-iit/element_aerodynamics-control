import scipy as sp
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.onnx
import torch.jit
from torch.autograd import Variable
import random as random
import time as time
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error


############################ PATH DEFINITIONS ###############################
matFilePath = pathlib.Path(__file__).parents[1] / "src" / "datasetFullAeroFrame.mat"
nnModelPath = pathlib.Path(__file__).parents[1] / "models" / "model_L9_N10_p1_60000_aeroFrame.pt" 

############################## DEVICE SETUP #################################
# Device will determine whether to run the training on GPU or CPU.
# device = torch.device('cpu')
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

########################## LOAD DATASET VARIABLES ###########################

# Load dataset .mat file
dataset = sp.io.loadmat(matFilePath)

# Load variables from dataset
cfdLinkNames = dataset['cfdLinkNames']
pitchAngle = dataset['pitchAngles_full']
yawAngle = dataset['yawAngles_full']
windDirection = dataset['windDirection_full']
jointPos = dataset['jointPosDeg_full']
linkCdAs = dataset['linkCdAs_matrix']
linkClAs = dataset['linkClAs_matrix']
linkCsAs = dataset['linkCsAs_matrix']
linkAoAs = dataset['linkAoAs_matrix']

linkAeroForces = np.concatenate((linkCdAs, linkClAs, linkCsAs), axis=1)

# Split variables for training and validation
datasetSplittingSeed = 56

pitchAngle_train, pitchAngle_val = train_test_split(pitchAngle, test_size=0.2, random_state=datasetSplittingSeed)
yawAngle_train, yawAngle_val = train_test_split(yawAngle, test_size=0.2, random_state=datasetSplittingSeed)
windDirection_train, windDirection_val = train_test_split(windDirection, test_size=0.2, random_state=datasetSplittingSeed)
jointPos_train, jointPos_val = train_test_split(jointPos, test_size=0.2, random_state=datasetSplittingSeed)
linkAoAs_train, linkAoAs_val = train_test_split(linkAoAs, test_size=0.2, random_state=datasetSplittingSeed)

linkAeroForces_train, linkAeroForces_val = train_test_split(linkAeroForces, test_size=0.2, random_state=datasetSplittingSeed)

# from arrays to tensors
pitchAngle_train = Variable(torch.from_numpy(pitchAngle_train.transpose()).float(), requires_grad=True)
pitchAngle_val = Variable(torch.from_numpy(pitchAngle_val.transpose()).float(), requires_grad=True)
yawAngle_train = Variable(torch.from_numpy(yawAngle_train.transpose()).float(), requires_grad=True)
yawAngle_val = Variable(torch.from_numpy(yawAngle_val.transpose()).float(), requires_grad=True)
windDirection_train = Variable(torch.from_numpy(windDirection_train.transpose()).float(), requires_grad=True)
windDirection_val = Variable(torch.from_numpy(windDirection_val.transpose()).float(), requires_grad=True)
jointPos_train = Variable(torch.from_numpy(jointPos_train.transpose()).float(), requires_grad=True)
jointPos_val = Variable(torch.from_numpy(jointPos_val.transpose()).float(), requires_grad=True)
linkAoAs_train = Variable(torch.from_numpy(linkAoAs_train.transpose()).float(), requires_grad=True)
linkAoAs_val = Variable(torch.from_numpy(linkAoAs_val.transpose()).float(), requires_grad=True)

linkAeroForces_train = Variable(torch.from_numpy(linkAeroForces_train.transpose()).float(), requires_grad=True)
linkAeroForces_val  = Variable(torch.from_numpy(linkAeroForces_val.transpose()).float(), requires_grad=True)

# Move tensors to the configured device
pitchAngle_train = pitchAngle_train.to(device)  
pitchAngle_val = pitchAngle_val.to(device) 
yawAngle_train = yawAngle_train.to(device)  
yawAngle_val = yawAngle_val.to(device)  
windDirection_train = windDirection_train.to(device)
windDirection_val = windDirection_val.to(device)
jointPos_train = jointPos_train.to(device)  
jointPos_val = jointPos_val.to(device)
linkAoAs_train = linkAoAs_train.to(device)
linkAoAs_val = linkAoAs_val.to(device)

linkAeroForces_train = linkAeroForces_train.to(device)  
linkAeroForces_val = linkAeroForces_val.to(device)

############################# LOAD THE NN MODEL #################################      

# model = torch.load(nnModelPath)
model = torch.jit.load(nnModelPath)
model.eval()
print('\n model loaded from: {}'.format(nnModelPath))

########################## COMPUTE THE NN MODEL OUTPUT #########################

linkAeroForces_predicted_train = model(windDirection_train, jointPos_train)
linkAeroForces_predicted_val  = model(windDirection_val, jointPos_val)

############################# PLOT THE RESULTS #################################
plotVariableName = r'$C_D A$'

nLink = len(cfdLinkNames[0][:])
plotVariable = 0 if plotVariableName == '$C_D A$' else 1 if plotVariableName == '$C_L A$' else 2    # 0: CdA, 1: ClA, 2: CsA
plotPreStartIndex = plotVariable*nLink

# Transfer data back to CPU if CUDA is active
if torch.cuda.is_available():
    pitchAngle_train = pitchAngle_train.cpu()
    pitchAngle_val   = pitchAngle_val.cpu()
    linkAoAs_train = linkAoAs_train.cpu()
    linkAoAs_val   = linkAoAs_val.cpu()
    linkAeroForces_train = linkAeroForces_train.cpu()
    linkAeroForces_val   = linkAeroForces_val.cpu()
    linkAeroForces_predicted_train = linkAeroForces_predicted_train.cpu()
    linkAeroForces_predicted_val   = linkAeroForces_predicted_val.cpu()


linkAoAs_train_plot = linkAoAs_train.detach().numpy()[0,:]
linkCdAs_train_plot = linkAeroForces_train.detach().numpy()[0,:]
linkAoAs_val_plot = linkAoAs_val.detach().numpy()[0,:]
linkCdAs_val_plot = linkAeroForces_val.detach().numpy()[0,:]


# plot
font_size = 40

plt.rcParams.update({'text.usetex': True,
                     'font.family': 'serif',
                     'font.size': font_size,
                     'font.weight': 'bold',
                     'font.style': 'italic'})
plt.figure(figsize=(20, 10))
plt.scatter(linkAoAs_train_plot, linkCdAs_train_plot, s=32, label='CFD train dataset', facecolors='none', edgecolors='tab:blue', alpha=0.5, linewidths=2)
plt.scatter(linkAoAs_val_plot, linkCdAs_val_plot, s=32, label='CFD validation dataset', facecolors='none', edgecolors='tab:orange', alpha=0.5, linewidths=2)
plt.xlabel(r'$\alpha_{link}$ [deg]')
plt.ylabel(r"$C_D A$")
# plt.ylim([0.1, 0.4])
# plt.xlim([0, 10])
plt.grid()
plt.legend()
# plt.show(block=False)
# manager = plt.get_current_fig_manager()
# manager.window.showMaximized()

# save image to pdf file with screen size
cwd = pathlib.Path(__file__).parents[0]
plt.savefig(str(cwd/'head-NN-1.pdf'), format='pdf')

print('debugging')



for linkIndex in range(0,nLink,1):

    plotStartIndex = plotPreStartIndex + linkIndex
    
    # Initialize the figure
    fig = plt.figure(figsize=(12, 10))
    
    # Subplot 1
    ax1 = fig.add_subplot(221)
    # ax1.scatter(linkAoAs[:,linkIndex], linkCdAs[:,linkIndex], s=16, label='CFD data', facecolors='none', edgecolors='tab:blue', alpha=0.5)
    ax1.scatter(linkAoAs_train.detach().numpy()[linkIndex,:], linkAeroForces_train.detach().numpy()[plotStartIndex,:], s=16, label='CFD train dataset', facecolors='none', edgecolors='tab:blue', alpha=0.5)
    ax1.scatter(linkAoAs_val.detach().numpy()[linkIndex,:], linkAeroForces_val.detach().numpy()[plotStartIndex,:], s=16, label='CFD validation dataset', facecolors='none', edgecolors='tab:orange', alpha=0.5)

    ax1.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax1.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax1.xaxis.label.set_fontsize(12)
    ax1.set_xlim([0,180])

    ax1.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
    ax1.set_ylabel(plotVariableName)
    ax1.yaxis.label.set_fontsize(12)
    ax1.yaxis.set_tick_params()
    limits = ax1.get_ylim()

    ax1.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax1.grid()
    ax1.legend()

    # Subplot 2
    ax2 = fig.add_subplot(222)
    ax2.scatter(linkAoAs_train.detach().numpy()[linkIndex,:], linkAeroForces_predicted_train.detach().numpy()[plotStartIndex,:], s=16, label='NN train prediction', facecolors='none', edgecolors='tab:green', alpha=0.5)
    ax2.scatter(linkAoAs_val.detach().numpy()[linkIndex,:], linkAeroForces_predicted_val.detach().numpy()[plotStartIndex,:], s=16, label='NN validation prediction', facecolors='none', edgecolors='tab:red', alpha=0.5)
    
    ax2.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax2.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax2.xaxis.label.set_fontsize(12)
    ax2.set_xlim([0,180])

    # ax2.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
    # ax2.set_ylabel(r'$C_D A$')
    #ax2.yaxis.label.set_fontsize(12)
    ax2.yaxis.set_tick_params()
    ax2.set_ylim(limits)

    ax2.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax2.grid()
    ax2.legend()

    
    # Subplot 3
    ax3 = fig.add_subplot(223)
    ax3.scatter(linkAoAs_train.detach().numpy()[linkIndex,:], linkAeroForces_predicted_train.detach().numpy()[plotStartIndex,:] - linkAeroForces_train.detach().numpy()[plotStartIndex,:], s=16, label='NN train error', facecolors='none', edgecolors='tab:green', alpha=0.5)
    ax3.scatter(linkAoAs_val.detach().numpy()[linkIndex,:], linkAeroForces_predicted_val.detach().numpy()[plotStartIndex,:] - linkAeroForces_val.detach().numpy()[plotStartIndex,:], s=16, label='NN validation error', facecolors='none', edgecolors='tab:red', alpha=0.5)
    
    ax3.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax3.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax3.xaxis.label.set_fontsize(12)
    ax3.set_xlim([0,180])

    ax3.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
    ax3.set_ylabel(r'$\Delta$ ' + plotVariableName)
    ax3.yaxis.label.set_fontsize(12)
    ax3.yaxis.set_tick_params()

    ax3.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax3.grid()
    ax3.legend()
    
    plt.show(block=False)
    
    # Print MSE
    trainMSE = mean_squared_error(linkAeroForces_predicted_train.detach().numpy()[plotStartIndex,:], linkAeroForces_train.detach().numpy()[plotStartIndex,:])
    # print(str(plotVariableName) + ' MSE for link ' + str(cfdLinkNames[0][linkIndex][0]) + ' on train dataset: ' + str(trainMSE))
    valMSE = mean_squared_error(linkAeroForces_predicted_val.detach().numpy()[plotStartIndex,:], linkAeroForces_val.detach().numpy()[plotStartIndex,:])
    # print(str(plotVariableName) + ' MSE for link ' + str(cfdLinkNames[0][linkIndex][0]) + ' on validation dataset: ' + str(valMSE))
    print("%.2e" % valMSE)


sumStartIndex = plotPreStartIndex
sumEndIndex   = plotPreStartIndex + nLink

# Initialize the figure
fig = plt.figure(figsize=(12, 10))

# Subplot 1
ax1 = fig.add_subplot(221)
ax1.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_train.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), s=16, label='train dataset', facecolors='none', edgecolors='tab:blue', alpha=0.5)
ax1.scatter(pitchAngle_val.detach().numpy()[0], np.sum(linkAeroForces_val.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), s=16, label='validation dataset', facecolors='none', edgecolors='tab:orange', alpha=0.5)
ax1.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax1.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax1.xaxis.label.set_fontsize(12)
ax1.set_xlim([0,180])
ax1.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax1.set_ylabel(plotVariableName)
ax1.yaxis.label.set_fontsize(12)
ax1.yaxis.set_tick_params()
limits = ax1.get_ylim()
ax1.set_title('iRonCub')
ax1.grid()
ax1.legend()

# Subplot 2
ax2 = fig.add_subplot(222)
ax2.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_predicted_train.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), s=16, label='NN train prediction', facecolors='none', edgecolors='tab:green', alpha=0.5)
ax2.scatter(pitchAngle_val.detach().numpy()[0], np.sum(linkAeroForces_predicted_val.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), s=16, label='NN validation prediction', facecolors='none', edgecolors='tab:red', alpha=0.5)
ax2.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax2.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax2.xaxis.label.set_fontsize(12)
ax2.set_xlim([0,180])
# ax2.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
# ax2.set_ylabel(r'$C_D A$')
# ax2.yaxis.label.set_fontsize(12)
ax2.yaxis.set_tick_params()
ax2.set_ylim(limits)
ax2.set_title('iRonCub')
ax2.grid()
ax2.legend()

# Subplot 3
ax3 = fig.add_subplot(223)
ax3.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_predicted_train.detach().numpy()[sumStartIndex:sumEndIndex,:] - linkAeroForces_train.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), 
            s=16, label='NN train error', facecolors='none', edgecolors='tab:green', alpha=0.5)
ax3.scatter(pitchAngle_val.detach().numpy()[0], np.sum(linkAeroForces_predicted_val.detach().numpy()[sumStartIndex:sumEndIndex,:] - linkAeroForces_val.detach().numpy()[sumStartIndex:sumEndIndex,:],axis=0), 
            s=16, label='NN validation error', facecolors='none', edgecolors='tab:red', alpha=0.5)
ax3.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax3.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax3.xaxis.label.set_fontsize(12)
ax3.set_xlim([0,180])
ax3.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax3.set_ylabel(r'$\Delta$ ' + plotVariableName)
ax3.yaxis.label.set_fontsize(12)
ax3.yaxis.set_tick_params()
# ax3.set_ylim(limits)
ax3.set_title('iRonCub')
ax3.grid()
ax3.legend()

plt.show(block=False)

# Closing all the plots
wait = input("Press Enter to close the figures.")
