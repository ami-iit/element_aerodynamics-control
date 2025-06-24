import scipy as sp
import pathlib
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.jit
from torch.autograd import Variable
import random as random
import time as time
from sklearn.model_selection import train_test_split


def main():
    ############################ PATH DEFINITIONS ###############################
    matFilePath = pathlib.Path(__file__).parents[1] / "src" / "datasetFullAeroFrame.mat"
    nnModelPath = (
        pathlib.Path(__file__).parents[1]
        / "models"
        / "model_L9_N10_p1_60000_aeroFrame.pt"
    )

    ############################## DEVICE SETUP #################################
    # Device will determine whether to run the training on GPU or CPU.
    # device = torch.device('cpu')
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    ########################## LOAD DATASET VARIABLES ###########################

    # Load dataset .mat file
    dataset = sp.io.loadmat(matFilePath)

    # Load variables from dataset
    cfdLinkNames = dataset["cfdLinkNames"]
    pitchAngle = dataset["pitchAngles_full"]
    yawAngle = dataset["yawAngles_full"]
    windDirection = dataset["windDirection_full"]
    jointPos = dataset["jointPosDeg_full"]
    linkCdAs = dataset["linkCdAs_matrix"]
    linkClAs = dataset["linkClAs_matrix"]
    linkCsAs = dataset["linkCsAs_matrix"]
    linkAoAs = dataset["linkAoAs_matrix"]

    linkAeroForces = np.concatenate((linkCdAs, linkClAs, linkCsAs), axis=1)

    # Split variables for training and validation
    datasetSplittingSeed = 56

    pitchAngle_train, pitchAngle_val = train_test_split(
        pitchAngle, test_size=0.2, random_state=datasetSplittingSeed
    )
    yawAngle_train, yawAngle_val = train_test_split(
        yawAngle, test_size=0.2, random_state=datasetSplittingSeed
    )
    windDirection_train, windDirection_val = train_test_split(
        windDirection, test_size=0.2, random_state=datasetSplittingSeed
    )
    jointPos_train, jointPos_val = train_test_split(
        jointPos, test_size=0.2, random_state=datasetSplittingSeed
    )
    linkAoAs_train, linkAoAs_val = train_test_split(
        linkAoAs, test_size=0.2, random_state=datasetSplittingSeed
    )

    linkAeroForces_train, linkAeroForces_val = train_test_split(
        linkAeroForces, test_size=0.2, random_state=datasetSplittingSeed
    )

    # from arrays to tensors
    pitchAngle_train = Variable(
        torch.from_numpy(pitchAngle_train.transpose()).float(), requires_grad=True
    )
    pitchAngle_val = Variable(
        torch.from_numpy(pitchAngle_val.transpose()).float(), requires_grad=True
    )
    yawAngle_train = Variable(
        torch.from_numpy(yawAngle_train.transpose()).float(), requires_grad=True
    )
    yawAngle_val = Variable(
        torch.from_numpy(yawAngle_val.transpose()).float(), requires_grad=True
    )
    windDirection_train = Variable(
        torch.from_numpy(windDirection_train.transpose()).float(), requires_grad=True
    )
    windDirection_val = Variable(
        torch.from_numpy(windDirection_val.transpose()).float(), requires_grad=True
    )
    jointPos_train = Variable(
        torch.from_numpy(jointPos_train.transpose()).float(), requires_grad=True
    )
    jointPos_val = Variable(
        torch.from_numpy(jointPos_val.transpose()).float(), requires_grad=True
    )
    linkAoAs_train = Variable(
        torch.from_numpy(linkAoAs_train.transpose()).float(), requires_grad=True
    )
    linkAoAs_val = Variable(
        torch.from_numpy(linkAoAs_val.transpose()).float(), requires_grad=True
    )

    linkAeroForces_train = Variable(
        torch.from_numpy(linkAeroForces_train.transpose()).float(), requires_grad=True
    )
    linkAeroForces_val = Variable(
        torch.from_numpy(linkAeroForces_val.transpose()).float(), requires_grad=True
    )

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
    print("\n model loaded from: {}".format(nnModelPath))

    ########################## COMPUTE THE NN MODEL OUTPUT #########################

    linkAeroForces_predicted_train = model(windDirection_train, jointPos_train)
    linkAeroForces_predicted_val = model(windDirection_val, jointPos_val)

    ############################# PLOT THE RESULTS #################################

    # Transfer data back to CPU if CUDA is active
    if torch.cuda.is_available():
        pitchAngle_train = pitchAngle_train.cpu()
        pitchAngle_val = pitchAngle_val.cpu()
        linkAoAs_train = linkAoAs_train.cpu()
        linkAoAs_val = linkAoAs_val.cpu()
        linkAeroForces_train = linkAeroForces_train.cpu()
        linkAeroForces_val = linkAeroForces_val.cpu()
        linkAeroForces_predicted_train = linkAeroForces_predicted_train.cpu()
        linkAeroForces_predicted_val = linkAeroForces_predicted_val.cpu()

    linkAoAs_train_plot = linkAoAs_train.detach().numpy()[0, :]
    linkCdAs_train_plot = linkAeroForces_train.detach().numpy()[0, :]
    linkCdAs_predicted_train_plot = linkAeroForces_predicted_train.detach().numpy()[
        0, :
    ]
    linkAoAs_val_plot = linkAoAs_val.detach().numpy()[0, :]
    linkCdAs_val_plot = linkAeroForces_val.detach().numpy()[0, :]
    linkCdAs_predicted_val_plot = linkAeroForces_predicted_val.detach().numpy()[0, :]

    # plot
    font_size = 60
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "serif",
            "font.size": font_size,
            "font.weight": "bold",
            "font.style": "italic",
        }
    )
    # Figure 1: AoA vs C_D A for training and validation data
    scatter_plot(
        x1=linkAoAs_train_plot,
        y1=linkCdAs_train_plot,
        x2=linkAoAs_val_plot,
        y2=linkCdAs_val_plot,
        label1="Training data",
        label2="Validation data",
        ylabel=r"$C_D A$",
        fig_name="head-NN-1.pdf",
    )
    # Figure 2: AoA vs C_D A for training and validation data with predictions
    scatter_plot(
        x1=linkAoAs_train_plot,
        y1=linkCdAs_predicted_train_plot,
        x2=linkAoAs_val_plot,
        y2=linkCdAs_predicted_val_plot,
        label1="Training predictions",
        label2="Validation predictions",
        ylabel=r"$C_D A$",
        fig_name="head-NN-2.pdf",
    )
    # Figure 3: AoA vs C_D A for training and validation data with predictions and errors
    scatter_plot(
        x1=linkAoAs_train_plot,
        y1=linkCdAs_predicted_train_plot - linkCdAs_train_plot,
        x2=linkAoAs_val_plot,
        y2=linkCdAs_predicted_val_plot - linkCdAs_val_plot,
        label1="Training errors",
        label2="Validation errors",
        ylabel=r"$\Delta C_D A$",
        fig_name="head-NN-3.pdf",
    )


def scatter_plot(x1, y1, x2, y2, label1, label2, ylabel, ylim, fig_name):
    plt.figure(figsize=(20, 12))
    plt.scatter(
        x1,
        y1,
        s=128,
        label=label1,
        facecolors="none",
        edgecolors="tab:blue",
        alpha=0.5,
        linewidths=3,
    )
    plt.scatter(
        x2,
        y2,
        s=128,
        label=label2,
        facecolors="none",
        edgecolors="tab:orange",
        alpha=0.5,
        linewidths=3,
    )
    plt.xlabel(r"$\alpha_{link}$ [deg]")
    plt.ylabel(ylabel)
    plt.grid()
    # make the legend in north east positions
    leg = plt.legend(markerscale=2)
    axis = plt.gca()
    pos = axis.get_position()
    axis.set_position([pos.x0 + 0.05, pos.y0 + 0.05, pos.width, pos.height + 0.05])
    for line in leg.legend_handles:
        line.set_linewidth(6)
    # save image to pdf file with screen size
    cwd = pathlib.Path(__file__).parents[0]
    plt.savefig(str(cwd / fig_name), format="pdf")
    plt.close()


if __name__ == "__main__":
    main()
