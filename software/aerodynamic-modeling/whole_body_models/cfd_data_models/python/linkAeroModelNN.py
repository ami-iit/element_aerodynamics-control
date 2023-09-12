import scipy as sp
import scipy.io
import pathlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import torch
import torch.nn as nn
import torch.onnx
import torch.jit
from torch.autograd import Variable
import random as random
import time as time
import torch.nn.init as init
import torch.nn.functional as F
from sklearn.model_selection import train_test_split


############################ PATH DEFINITIONS ###############################
matFilePath = pathlib.Path(__file__).parents[1] / "src" / "datasetAlias.mat"
fakeMatFilePath = pathlib.Path(__file__).parents[1] / "src" / "datasetFake.mat"
nnModelPath = pathlib.Path(__file__).parents[1] / "src" / "model.pt" 

############################## DEVICE SETUP #################################
# Device will determine whether to run the training on GPU or CPU.
device = torch.device('cpu') # alternative for cuda systems: torch.device('cuda' if torch.cuda.is_available() else 'cpu')

########################## LOAD DATASET VARIABLES ###########################
# Load dataset .mat file
dataset = sp.io.loadmat(matFilePath)

# Load variables from dataset
cfdLinkNames = dataset['cfdLinkNames']
pitchAngle = dataset['pitchAngles_full']
yawAngle = dataset['yawAngles_full']
jointPos = dataset['jointPosDeg_full']
linkCdAs = dataset['linkCdAs_matrix']
linkClAs = dataset['linkClAs_matrix']
linkCsAs = dataset['linkCsAs_matrix']
linkAoAs = dataset['linkAoAs_matrix']

linkAeroForces = np.concatenate((linkCdAs, linkClAs, linkCsAs), axis=1)

# Split variables for training and testing
datasetSplittingSeed = 56

pitchAngle_train, pitchAngle_test = train_test_split(pitchAngle, test_size=0.2, random_state=datasetSplittingSeed)
yawAngle_train, yawAngle_test = train_test_split(yawAngle, test_size=0.2, random_state=datasetSplittingSeed)
jointPos_train, jointPos_test = train_test_split(jointPos, test_size=0.2, random_state=datasetSplittingSeed)
linkAoAs_train, linkAoAs_test = train_test_split(linkAoAs, test_size=0.2, random_state=datasetSplittingSeed)

linkAeroForces_train, linkAeroForces_test = train_test_split(linkAeroForces, test_size=0.2, random_state=datasetSplittingSeed)

# from arrays to tensors
pitchAngle_train = Variable(torch.from_numpy(pitchAngle_train.transpose()).float(), requires_grad=True)
pitchAngle_test  = Variable(torch.from_numpy(pitchAngle_test.transpose()).float(), requires_grad=True)
yawAngle_train   = Variable(torch.from_numpy(yawAngle_train.transpose()).float(), requires_grad=True)
yawAngle_test    = Variable(torch.from_numpy(yawAngle_test.transpose()).float(), requires_grad=True)
jointPos_train   = Variable(torch.from_numpy(jointPos_train.transpose()).float(), requires_grad=True)
jointPos_test    = Variable(torch.from_numpy(jointPos_test.transpose()).float(), requires_grad=True)

linkAeroForces_train = Variable(torch.from_numpy(linkAeroForces_train.transpose()).float(), requires_grad=True)
linkAeroForces_test  = Variable(torch.from_numpy(linkAeroForces_test.transpose()).float(), requires_grad=True)

############################# LOAD FAKE DATASET #############################
# Load fake dataset .mat file
dataset_fake = sp.io.loadmat(fakeMatFilePath)

# Load variables from dataset
pitchAngle_fake = dataset_fake['pitchAngles_full']
yawAngle_fake = dataset_fake['yawAngles_full']
jointPos_fake = dataset_fake['jointPosDeg_full']
linkCdAs_fake = dataset_fake['linkCdAs_matrix']
linkClAs_fake = dataset_fake['linkClAs_matrix']
linkCsAs_fake = dataset_fake['linkCsAs_matrix']
linkAoAs_fake = dataset_fake['linkAoAs_matrix']

linkAeroForces_fake = np.concatenate((linkCdAs_fake, linkClAs_fake, linkCsAs_fake), axis=1)

# from arrays to tensors
pitchAngle_fake = Variable(torch.from_numpy(pitchAngle_fake.transpose()).float(), requires_grad=True)
yawAngle_fake   = Variable(torch.from_numpy(yawAngle_fake.transpose()).float(), requires_grad=True)
jointPos_fake   = Variable(torch.from_numpy(jointPos_fake.transpose()).float(), requires_grad=True)

linkAeroForces_fake = Variable(torch.from_numpy(linkAeroForces_fake.transpose()).float(), requires_grad=True)


############################# IDYNTREE CODE ###############################
# componentPath  = pathlib.Path("C:/Users/apaolino/code/component_ironcub")
# modelPath      = componentPath / "models" / "iRonCub-Mk1" / "iRonCub" / "robots" / "iRonCub-Mk1_Gazebo" 
# fileName       = 'model_stl.urdf'
# meshFilePrefix = componentPath / "models"
# jointNames     = ['torso_pitch','torso_roll','torso_yaw', 'l_shoulder_pitch', 'l_shoulder_roll','l_shoulder_yaw',
#                   'l_elbow', 'r_shoulder_pitch', 'r_shoulder_roll','r_shoulder_yaw','r_elbow',
#                   'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw','l_knee','r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee']


# ### Aerodynamic forces application points definitions
# aeroFrameNames = ['head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine',
#                   'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',
#                   'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg']

# cfdLinkNames   = ['head', 'torso', 'left_back_turbine', 'right_back_turbine',
#                   'left_arm','left_arm_turbine','right_arm','right_arm_turbine',
#                   'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower']


############################# LOAD THE NN MODEL #################################      

# model = torch.load(nnModelPath)
model = torch.jit.load(nnModelPath)
model.eval()

########################## COMPUTE THE NN MODEL OUTPUT #########################

linkAeroForces_predicted_train = model (pitchAngle_train, yawAngle_train, jointPos_train)
linkAeroForces_predicted_test  = model (pitchAngle_test, yawAngle_test, jointPos_test)
linkAeroForces_predicted_fake  = model (pitchAngle_fake, yawAngle_fake, jointPos_fake)

############################# PLOT THE RESULTS #################################
nLink = len(cfdLinkNames[0][:])

for linkIndex in range(0,nLink,1):

    # Initialize the figure
    fig = plt.figure(figsize=(12, 10))
    
    # Subplot 1
    ax1 = fig.add_subplot(221)
    # ax1.scatter(linkAoAs[:,linkIndex], linkCdAs[:,linkIndex], s=16, label='CFD data', facecolors='none', edgecolors='tab:blue', alpha=0.5)
    ax1.scatter(linkAoAs_train[:,linkIndex], linkAeroForces_train.detach().numpy()[linkIndex,:], s=16, label='CFD train dataset', facecolors='none', edgecolors='tab:blue', alpha=0.5)
    ax1.scatter(linkAoAs_test[:,linkIndex], linkAeroForces_test.detach().numpy()[linkIndex,:], s=16, label='CFD test dataset', facecolors='none', edgecolors='tab:orange', alpha=0.5)

    ax1.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax1.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax1.xaxis.label.set_fontsize(12)
    ax1.set_xlim([0,180])

    ax1.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
    ax1.set_ylabel(r'$C_D A$')
    ax1.yaxis.label.set_fontsize(12)
    ax1.yaxis.set_tick_params()
    limits = ax1.get_ylim()

    ax1.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax1.grid()
    ax1.legend()

    # Subplot 2
    ax2 = fig.add_subplot(222)
    ax2.scatter(linkAoAs_train[:,linkIndex], linkAeroForces_predicted_train.detach().numpy()[linkIndex,:], s=16, label='NN train prediction', facecolors='none', edgecolors='tab:green', alpha=0.5)
    ax2.scatter(linkAoAs_test[:,linkIndex], linkAeroForces_predicted_test.detach().numpy()[linkIndex,:], s=16, label='NN test prediction', facecolors='none', edgecolors='tab:red', alpha=0.5)
    
    ax2.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax2.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax2.xaxis.label.set_fontsize(12)
    ax2.set_xlim([0,180])

    #ax2.yaxis.set_label_coords(-0.12, 0.5)  # Adjust the y-axis label position
    #ax2.set_ylabel(r'$C_D A$')
    #ax2.yaxis.label.set_fontsize(12)
    ax2.yaxis.set_tick_params()
    ax2.set_ylim(limits)

    ax2.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax2.grid()
    ax2.legend()

    
    # Subplot 3
    ax3 = fig.add_subplot(223)
    ax3.scatter(linkAoAs_train[:,linkIndex], linkAeroForces_predicted_train.detach().numpy()[linkIndex,:] - linkAeroForces_train.detach().numpy()[linkIndex,:], s=16, label='NN train error', facecolors='none', edgecolors='tab:green', alpha=0.5)
    ax3.scatter(linkAoAs_test[:,linkIndex], linkAeroForces_predicted_test.detach().numpy()[linkIndex,:] - linkAeroForces_test.detach().numpy()[linkIndex,:], s=16, label='NN test error', facecolors='none', edgecolors='tab:red', alpha=0.5)
    
    ax3.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax3.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax3.xaxis.label.set_fontsize(12)
    ax3.set_xlim([0,180])

    #ax3.yaxis.set_label_coords(-0.12, 0.5)  # Adjust the y-axis label position
    #ax3.set_ylabel(r'$C_D A$')
    #ax3.yaxis.label.set_fontsize(12)
    ax3.yaxis.set_tick_params()
    # ax3.set_ylim(limits)

    ax3.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax3.grid()
    ax3.legend()
    
    
    # Subplot 4
    ax4 = fig.add_subplot(224)
    ax4.scatter(linkAoAs_fake[:,linkIndex], linkAeroForces_predicted_fake.detach().numpy()[linkIndex,:], s=16, label='NN fake prediction', facecolors='none', edgecolors='tab:purple', alpha=0.5)
    
    ax4.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
    ax4.set_xlabel(r'$\alpha_{link}$ [deg]')
    ax4.xaxis.label.set_fontsize(12)
    ax4.set_xlim([0,180])

    #ax4.yaxis.set_label_coords(-0.12, 0.5)  # Adjust the y-axis label position
    #ax4.set_ylabel(r'$C_D A$')
    #ax4.yaxis.label.set_fontsize(12)
    ax4.yaxis.set_tick_params()
    ax4.set_ylim(limits)

    ax4.set_title(str(cfdLinkNames[0][linkIndex][0]))
    ax4.grid()
    ax4.legend()

    plt.show(block=False)
    
    
# Initialize the figure
fig = plt.figure(figsize=(12, 10))

# Subplot 1
ax1 = fig.add_subplot(221)
ax1.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_train.detach().numpy()[:nLink,:],axis=0), s=16, label='train dataset', facecolors='none', edgecolors='tab:blue', alpha=0.5)
ax1.scatter(pitchAngle_test.detach().numpy()[0], np.sum(linkAeroForces_test.detach().numpy()[:nLink,:],axis=0), s=16, label='test dataset', facecolors='none', edgecolors='tab:orange', alpha=0.5)
ax1.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax1.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax1.xaxis.label.set_fontsize(12)
ax1.set_xlim([0,180])
ax1.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax1.set_ylabel(r'$C_D A$')
ax1.yaxis.label.set_fontsize(12)
ax1.yaxis.set_tick_params()
limits = ax1.get_ylim()
ax1.set_title('iRonCub')
ax1.grid()
ax1.legend()

# Subplot 2
ax2 = fig.add_subplot(222)
ax2.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_predicted_train.detach().numpy()[:nLink,:],axis=0), s=16, label='NN train prediction', facecolors='none', edgecolors='tab:green', alpha=0.5)
ax2.scatter(pitchAngle_test.detach().numpy()[0], np.sum(linkAeroForces_predicted_test.detach().numpy()[:nLink,:],axis=0), s=16, label='NN test prediction', facecolors='none', edgecolors='tab:red', alpha=0.5)
ax2.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax2.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax2.xaxis.label.set_fontsize(12)
ax2.set_xlim([0,180])
ax2.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax2.set_ylabel(r'$C_D A$')
ax2.yaxis.label.set_fontsize(12)
ax2.yaxis.set_tick_params()
ax2.set_ylim(limits)
ax2.set_title('iRonCub')
ax2.grid()
ax2.legend()

# Subplot 3
ax3 = fig.add_subplot(223)
ax3.scatter(pitchAngle_train.detach().numpy()[0], np.sum(linkAeroForces_predicted_train.detach().numpy()[:nLink,:] - linkAeroForces_train.detach().numpy()[:nLink,:],axis=0), 
            s=16, label='NN train error', facecolors='none', edgecolors='tab:green', alpha=0.5)
ax3.scatter(pitchAngle_test.detach().numpy()[0], np.sum(linkAeroForces_predicted_test.detach().numpy()[:nLink,:] - linkAeroForces_test.detach().numpy()[:nLink,:],axis=0), 
            s=16, label='NN test error', facecolors='none', edgecolors='tab:red', alpha=0.5)
ax3.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax3.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax3.xaxis.label.set_fontsize(12)
ax3.set_xlim([0,180])
ax3.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax3.set_ylabel(r'$C_D A$')
ax3.yaxis.label.set_fontsize(12)
ax3.yaxis.set_tick_params()
# ax3.set_ylim(limits)
ax3.set_title('iRonCub')
ax3.grid()
ax3.legend()

# Subplot 4
ax4 = fig.add_subplot(224)
ax4.scatter(pitchAngle_fake.detach().numpy()[0], np.sum(linkAeroForces_predicted_fake.detach().numpy()[:nLink,:],axis=0), 
            s=16, label='NN fake prediction', facecolors='none', edgecolors='tab:purple', alpha=0.5)
ax4.xaxis.set_label_coords(0.5, -0.08)  # Adjust the x-axis label position
ax4.set_xlabel(r'$\alpha_{robot}$ [deg]')
ax4.xaxis.label.set_fontsize(12)
ax4.set_xlim([0,180])
ax4.yaxis.set_label_coords(-0.15, 0.5)  # Adjust the y-axis label position
ax4.set_ylabel(r'$C_D A$')
ax4.yaxis.label.set_fontsize(12)
ax4.yaxis.set_tick_params()
# ax3.set_ylim(limits)
ax4.set_title('iRonCub')
ax4.grid()
ax4.legend()


# For displaying all the plots
plt.show()