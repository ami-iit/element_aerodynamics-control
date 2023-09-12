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


############################ SCRIPT COMMANDS ################################
TRAIN_NN_MODEL = True # True if you want to train the NN model
SAVE_NN_MODEL  = True # True if you want to save the NN model
LOAD_NN_MODEL  = not TRAIN_NN_MODEL # True if you want to load the NN model

############################ PATH DEFINITIONS ###############################
matFilePath = pathlib.Path(__file__).parents[1] / "src" / "datasetAlias.mat"
nnModelPath = pathlib.Path(__file__).parents[1] / "src" / "model.pt" 

# Device will determine whether to run the training on GPU or CPU.
device = torch.device('cpu') # alternative for cuda systems: torch.device('cuda' if torch.cuda.is_available() else 'cpu')

########################## LOAD DATASET VARIABLES ###########################

# Load dataset .mat file
dataset = sp.io.loadmat(matFilePath)

# Load variables from dataset
pitchAngle = dataset['pitchAngles_full']
yawAngle = dataset['yawAngles_full']
jointPos = dataset['jointPosDeg_full']
linkCdAs = dataset['linkCdAs_matrix']
linkClAs = dataset['linkClAs_matrix']
linkCsAs = dataset['linkCsAs_matrix']

linkAeroForces = np.concatenate((linkCdAs, linkClAs, linkCsAs), axis=1)

# Split variables for training and testing
datasetSplittingSeed = 56

pitchAngle_train, pitchAngle_test = train_test_split(pitchAngle, test_size=0.2, random_state=datasetSplittingSeed)
yawAngle_train, yawAngle_test = train_test_split(yawAngle, test_size=0.2, random_state=datasetSplittingSeed)
jointPos_train, jointPos_test = train_test_split(jointPos, test_size=0.2, random_state=datasetSplittingSeed)

linkAeroForces_train, linkAeroForces_test = train_test_split(linkAeroForces, test_size=0.2, random_state=datasetSplittingSeed)

# from arrays to tensors
pitchAngle_train = Variable(torch.from_numpy(pitchAngle_train.transpose()).float(), requires_grad=True)
yawAngle_train   = Variable(torch.from_numpy(yawAngle_train.transpose()).float(), requires_grad=True)
jointPos_train   = Variable(torch.from_numpy(jointPos_train.transpose()).float(), requires_grad=True)

linkAeroForces_train = Variable(torch.from_numpy(linkAeroForces_train.transpose()).float(), requires_grad=True)


# Move tensors to the configured device
pitchAngle_train = pitchAngle_train.to(device)  # input
yawAngle_train = yawAngle_train.to(device)  # input
jointPos_train = jointPos_train.to(device)  # input

linkAeroForces_train = linkAeroForces_train.to(device)  #  CFD data

###################### SOME PARAMETERS OF THE NN #########################
seme = 1000 # seed for cuda
batch_size = 1000
num_epochs = 5000
learning_rate = 0.001 
input_parameters = 21
n_neurons = 21*4 # neurons for each layer
output_parameters = 39

################################ NN CLASS ################################
class Net(nn.Module):     # 3 layers in standard simulation
    def __init__(self):
        super(Net, self).__init__()
        self.input_layer = nn.Linear(input_parameters,n_neurons)
        self.hidden_layer1 = nn.Linear(n_neurons,n_neurons)
        self.hidden_layer2 = nn.Linear(n_neurons,n_neurons)
        self.hidden_layer3 = nn.Linear(n_neurons,n_neurons)
        self.hidden_layer4 = nn.Linear(n_neurons,n_neurons)
        self.hidden_layer5 = nn.Linear(n_neurons,n_neurons)
        self.hidden_layer6 = nn.Linear(n_neurons,n_neurons)
        self.output_layer = nn.Linear(n_neurons,output_parameters)
    def forward(self, pitchAngle, yawAngle, jointPos):
        pitchAngle = pitchAngle#.unsqueeze(0)
        yawAngle = yawAngle#.unsqueeze(0)
        torso_pitch = jointPos[0,:].unsqueeze(0)
        torso_roll = jointPos[1,:].unsqueeze(0)
        torso_yaw = jointPos[2,:].unsqueeze(0)
        l_shoulder_pitch = jointPos[3,:].unsqueeze(0)
        l_shoulder_roll = jointPos[4,:].unsqueeze(0)
        l_shoulder_yaw = jointPos[5,:].unsqueeze(0)
        l_elbow = jointPos[6,:].unsqueeze(0)
        r_shoulder_pitch = jointPos[7,:].unsqueeze(0)
        r_shoulder_roll = jointPos[8,:].unsqueeze(0)
        r_shoulder_yaw = jointPos[9,:].unsqueeze(0)
        r_elbow = jointPos[10,:].unsqueeze(0)
        l_hip_pitch = jointPos[11,:].unsqueeze(0)
        l_hip_roll = jointPos[12,:].unsqueeze(0)
        l_hip_yaw = jointPos[13,:].unsqueeze(0)
        l_knee = jointPos[14,:].unsqueeze(0)
        r_hip_pitch = jointPos[15,:].unsqueeze(0)
        r_hip_roll = jointPos[16,:].unsqueeze(0)
        r_hip_yaw = jointPos[17,:].unsqueeze(0)
        r_knee = jointPos[18,:].unsqueeze(0)
        input = torch.cat([pitchAngle, yawAngle,
                           torso_pitch, torso_roll, torso_yaw,
                           l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,
                           r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,
                           l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,
                           r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee],dim=0)
        input_layer_out = F.relu(self.input_layer(input.T))
        layer1_out = F.relu(self.hidden_layer1(input_layer_out))
        layer2_out = F.relu(self.hidden_layer2(layer1_out))
        layer3_out = F.relu(self.hidden_layer3(layer2_out))
        layer4_out = F.relu(self.hidden_layer4(layer3_out))
        layer5_out = F.relu(self.hidden_layer5(layer4_out))
        layer6_out = F.relu(self.hidden_layer6(layer5_out))
        output = self.output_layer(layer6_out)
        return output.T

########################### TRAINING OPERATIONS ##############################
if TRAIN_NN_MODEL:
    
    ################################ NN INIT #################################
    torch.manual_seed(seme)  # fix the seed for neural network
    if torch.cuda.is_available():
      torch.cuda.manual_seed_all(seme)

    w = torch.empty(input_parameters, n_neurons)
    nn.init.xavier_normal_(w)

    model = Net().to(device)

    print(model) # Verify the initialization of hyperparameters

    mse_cost_function = torch.nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    ########################### TRAINING OF THE NN ###########################
    train_loss = []
    start_training = time.time()

    for epoch in range(num_epochs):

        outputs = model(pitchAngle_train, yawAngle_train, jointPos_train)
        loss = mse_cost_function(outputs, linkAeroForces_train)

        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        train_loss.append(loss.data.item())

        print('Epoch [{}/{}], Loss: {:.16f}'.format(epoch +
              1, num_epochs, loss.item()))


    time_training = time.time() - start_training
    print('time_training [min]')
    print(time_training/60)


    ######################### VISUALIZING TRAINING DATA #########################
    plt.figure(figsize=(8, 6))
    epochs = np.arange(num_epochs)

    plt.plot(epochs,train_loss,label="Train loss")
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.title('Loss')
    plt.grid()
    plt.legend()
    plt.show(block=False)

    plt.figure(figsize=(8, 6))

    plt.semilogy(epochs,train_loss,label="Train loss")
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.title('Loss')
    plt.grid()
    plt.legend()
    plt.show(block=False)

    #############################################################################

    ########################## SAVE THE NN MODEL ################################
    if SAVE_NN_MODEL:
        # torch.save(model, nnModelPath)
        model_scripted = torch.jit.script(model) # Export to TorchScript
        model_scripted.save(nnModelPath) # Save        
     
    #############################################################################

############################# LOAD THE NN MODEL #################################      
else:
    # model = torch.load(nnModelPath)
    model = torch.jit.load(nnModelPath)
    model.eval()


######################### RESULT VERIFICATION #########################
linkAeroForces_predicted_train = model (pitchAngle_train, yawAngle_train, jointPos_train)

train_error = linkAeroForces_predicted_train - linkAeroForces_train
train_square_error = torch.pow(train_error,2)
train_mean_square_error = torch.mean(train_square_error)
print('Train verification MSE:')
print(train_mean_square_error)



### TEST ###

# from arrays to tensors
pitchAngle_test = Variable(torch.from_numpy(pitchAngle_test.transpose()).float(), requires_grad=True)
yawAngle_test   = Variable(torch.from_numpy(yawAngle_test.transpose()).float(), requires_grad=True)
jointPos_test   = Variable(torch.from_numpy(jointPos_test.transpose()).float(), requires_grad=True)

linkAeroForces_test  = Variable(torch.from_numpy(linkAeroForces_test.transpose()).float(), requires_grad=True)

# Move tensors to the configured device
pitchAngle_test = pitchAngle_test.to(device)  # input
yawAngle_test = yawAngle_test.to(device)  # input
jointPos_test = jointPos_test.to(device)  # input

linkAeroForces_test = linkAeroForces_test.to(device)  #  CFD data


linkAeroForces_predicted_test = model (pitchAngle_test, yawAngle_test, jointPos_test)

test_error = linkAeroForces_predicted_test - linkAeroForces_test
test_square_error = torch.pow(test_error,2)
test_mean_square_error = torch.mean(test_square_error)
print('Test MSE:')
print(test_mean_square_error)

plt.show()
