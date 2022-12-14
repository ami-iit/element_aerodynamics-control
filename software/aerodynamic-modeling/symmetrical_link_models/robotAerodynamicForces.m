close all; clear all; clc;

%% robot data init
jointNames = {'torso_pitch','torso_roll','torso_yaw', ...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

frameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
    'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
    'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

componentPath  = getenv('IRONCUB_COMPONENT_SOURCE_DIR');
modelPath      = [componentPath,'\models\iRonCub-Mk1\iRonCub\robots\iRonCub-Mk1_Gazebo\'];
fileName       = 'model_stl.urdf';
meshFilePrefix = [componentPath,'\models'];

%% ANALYSIS TYPE
TEST = 'hovering'; % | hovering | flight30 | flight50 | flight60 |

if matches(TEST,'hovering')
    pitchAngle  = 90;
    yawAngles   = 0:5:90;
    Npoints     = length(yawAngles);
    jointPos    = [0,0,0,-10,25,40,15,-10,25,40,15,0,10,7,0,0,0,0,10,7,0,0,0]* pi/180;
elseif matches(TEST,'flight30')
    pitchAngles = 25:5:65;
    yawAngle    = 0;
    Npoints     = length(pitchAngles);
    jointPos    = [0,0,0,-40.7,11.3,26.5,58.3,-40.7,11.3,26.5,58.3,0,10,7,0,0,0,0,10,7,0,0,0]* pi/180;
elseif matches(TEST,'flight50')
    pitchAngles = 25:5:65;
    yawAngle    = 0;
    Npoints     = length(pitchAngles);
    jointPos    = [0,0,0,-31.3,19,26.3,45.3,-31.3,19,26.3,45.3,0,10,7,0,0,0,0,10,7,0,0,0]* pi/180;
elseif matches(TEST,'flight60')
    pitchAngles = 25:5:65;
    yawAngle    = 0;
    Npoints     = length(pitchAngles);
    jointPos    = [0,0,0,-25,24,30,35,-25,24,30,35,0,10,7,0,0,0,0,10,7,0,0,0]* pi/180;
end

%% initialize plot variables
totalCdA = nan(Npoints,1);
robotCdA = nan(Npoints,1);
totalCsA = nan(Npoints,1);
robotCsA = nan(Npoints,1);
totalClA = nan(Npoints,1);
robotClA = nan(Npoints,1);

%% Start test point cycles
for j = 1:Npoints

    %% airflow input
    if contains(TEST,'hovering')
        yawAngle   = yawAngles(j);  % [deg]
        barAngle   = 0; % [deg]
    elseif contains(TEST,'flight')
        pitchAngle = pitchAngles(j);  % [deg]
        barAngle   = 45; % [deg]
    end
    airSpeed               = 17;            % [m/s]
    airDensity             = 1.225;         % [kg/m^3]
    airDynamicViscosity    = 1.8e-5;        % [N s/m^2] at T = 18 C
    
    % wind tunnel frame relative velocity
    w_relativeVelocity     = [-1; 0; 0] *airSpeed;

    %% robot simplified model data (sphere and cylinders links)
    linkDiameters    = [0.1929, 0.2467, 0.102, 0.102, ...
                        0.0864, 0.0845, 0.0864, 0.0845, ...
                        0.1476, 0.1256, 0.1197, 0.1256, 0.1197];
    linkLengths      = [0.1929, 0.1734, 0.2955, 0.2955, ...
                        0.135, 0.223, 0.135, 0.223, ...
                        0.2282, 0.1573, 0.2351, 0.1573, 0.2351];
    linkAspectRatios = linkLengths./linkDiameters;
    linkRefAreas     = linkLengths.*linkDiameters;
    linkRefAreas(1)  = (pi/4)*linkDiameters(1)^2;


    %% set robot state

    % set base Pose according to yaw and pitch angles
    R_yaw     = rotz(yawAngle);
    R_pitch   = roty(pitchAngle - 90);
    basePose  = [R_yaw * R_pitch, [0.3; 0; 0];
                      zeros(1,3),          1];
    % set bar axis versor
    R_bar           = R_yaw * R_pitch * roty(barAngle);
    w_barAxisVersor = R_bar * [-1; 0; 0];

    % data for using iDynTreeWrappers functions
    jointVel = zeros(23,1);
    baseVel  = zeros(6,1);
    gravAcc  = [0; 0; 9.81];

    %% robot initialization
    KinDynModel = iDynTreeWrappers.loadReducedModel(jointNames, 'root_link', modelPath, fileName, false);
    iDynTreeWrappers.setRobotState(KinDynModel, basePose, jointPos, baseVel, jointVel, gravAcc);

    %% robot visualization
    % iDynTreeWrappers.prepareVisualization(KinDynModel, meshFilePrefix, 'color', [0.9,0.9,0.9], 'material', 'metal', ...
    %                                                      'transparency', 0.7, 'debug', true, 'view', [-45 20]);

    %% local axis versor evaluation
    Nlinks       = length(frameNames);

    w_axisVersor = nan(3,Nlinks);

    for i = 1:Nlinks
        if matches(frameNames{i},{'head','chest','root_link'})
            w_H_l = iDynTreeWrappers.getWorldTransform(KinDynModel,frameNames{i});
            w_axisVersor(:,i) = w_H_l(1:3,1:3) * [0; 1; 0];
        else
            w_H_l = iDynTreeWrappers.getWorldTransform(KinDynModel,frameNames{i});
            w_axisVersor(:,i) = w_H_l(1:3,1:3) * [0; 0; 1];
        end
    end

    %% aerodynamic coefficients evaluation with model functions
    
    % initialize variables
    angleOfAttack  = nan(Npoints,1);
    reynoldsNumber = nan(Npoints,1);
    Cd             = nan(Npoints,1);
    Cn             = nan(Npoints,1);
    Cn_sin         = nan(Npoints,1);
    
    % Evaluate Cd and Cn in local coordinate frames
    for i = 1:Nlinks
        angleOfAttack(i)  = acosd(abs((transpose(w_axisVersor(:,i))*w_relativeVelocity))/airSpeed); % [deg]
        reynoldsNumber(i) = (airDensity*airSpeed*linkDiameters(i))/airDynamicViscosity;
        if matches(frameNames{i},'head')
            [Cd(i), Cn(i)]        = sphereAerodynamicForces(reynoldsNumber(i));
        else
            [Cd(i), ~, Cn_sin(i)] = cylinderAerodynamicForces(angleOfAttack(i),reynoldsNumber(i),linkAspectRatios(i));
        end
    end

    %% aerodynamic force components calculation without support-bar
    
    % initialize variables
    linkDragForce   = nan(3,Nlinks);
    linkNormalForce = nan(3,Nlinks);
    
    % assign forces according to coefficients
    for i = 1:Nlinks
        linkDragForce(:, i) = - 0.5 * airDensity * linkRefAreas(i) * airSpeed * Cd(i) * w_relativeVelocity;

        if matches(frameNames{i},'head')
            linkNormalForce(:, i) = 0.5 * airDensity * linkRefAreas(i) * Cn(i) * ...
                                  sign(transpose(w_axisVersor(:,i))*w_relativeVelocity) * ...
                                  cross(cross(w_relativeVelocity,w_axisVersor(:,i)),w_relativeVelocity) ;
        else
            linkNormalForce(:, i) = 0.5 * airDensity * linkRefAreas(i) * Cn_sin(i) * ...
                                  sign(transpose(w_axisVersor(:,i))*w_relativeVelocity) * ...
                                  cross(cross(w_relativeVelocity,w_axisVersor(:,i)),w_relativeVelocity) ;
        end
    end

    robotDragForce   = sum(linkDragForce,2); % [N]
    robotNormalForce = sum(linkNormalForce,2); % [N]

    robotLiftForce   = [0; 0; robotNormalForce(3)]; % [N]
    robotSideForce   = [0; robotNormalForce(2); 0]; % [N]

    robotDragArea  = robotDragForce(1) / (0.5 * airDensity * airSpeed^2); % [m^2]
    robotLiftArea  = robotLiftForce(3) / (0.5 * airDensity * airSpeed^2); % [m^2]
    robotSideArea  = robotSideForce(2) / (0.5 * airDensity * airSpeed^2); % [m^2]


    %% aerodynamic force components calculation with support-bar

    % bar geometry values
    barDiameter     = 0.0547; % [m]
    barLength       = 0.8592; % [m]
    barAspectRatio  = barLength/barDiameter;
    barRefArea      = barLength*barDiameter; % [m^2]

    barAoA                  = acosd(abs((transpose(w_barAxisVersor)*w_relativeVelocity))/airSpeed); % [deg]
    barReynoldsNumber       = (airDensity*airSpeed*barDiameter)/airDynamicViscosity;
    [Cd_bar, ~, Cn_bar_sin] = cylinderAerodynamicForces(barAoA,barReynoldsNumber,barAspectRatio);

    barDragForce   = - 0.5 * airDensity * barRefArea * airSpeed * Cd_bar * w_relativeVelocity; % [N]
    barNormalForce = 0.5 * airDensity * barRefArea * Cn_bar_sin * ...
                     sign(transpose(w_barAxisVersor)*w_relativeVelocity) * ...
                     cross(cross(w_relativeVelocity,w_barAxisVersor),w_relativeVelocity); % [N]

    barLiftForce = [0; 0; barNormalForce(3)]; % [N]
    barSideForce = [0; barNormalForce(2); 0]; % [N]

    totalDragForce = robotDragForce + barDragForce; % [N]
    totalLiftForce = robotLiftForce + barLiftForce; % [N]
    totalSideForce = robotSideForce + barSideForce; % [N]

    totalDragArea = totalDragForce(1) / (0.5 * airDensity * airSpeed^2); % [m^2]
    totalSideArea = totalSideForce(2) / (0.5 * airDensity * airSpeed^2); % [m^2]
    totalLiftArea = totalLiftForce(3) / (0.5 * airDensity * airSpeed^2); % [m^2]

    %% Collect results for plots
    totalCdA(j) = totalDragArea;
    robotCdA(j) = robotDragArea;

    totalCsA(j) = totalSideArea;
    robotCsA(j) = robotSideArea;

    totalClA(j) = totalLiftArea;
    robotClA(j) = robotLiftArea;
end

%% PLOTS
if contains(TEST,'hovering')
    plotAngles = yawAngles;
    angleName  = '$\beta\,[^\circ]$';
elseif contains(TEST,'flight')
    plotAngles = pitchAngles;
    angleName  = '$\alpha\,[^\circ]$';
end

fig1 = figure();
plot(plotAngles,totalCdA,'k-','linewidth',1.5,'DisplayName','robot + support');hold on;
plot(plotAngles,robotCdA,'k:','linewidth',1.5,'DisplayName','robot');hold on;
grid on;
ylabel('$C_D A\,[m^2]$','Interpreter','latex')
xlabel(angleName,'Interpreter','latex')
legend('Interpreter','latex','Location','best')
legend show

fig2 = figure();
plot(plotAngles,totalClA,'k-','linewidth',1.5,'DisplayName','robot + support');hold on;
plot(plotAngles,robotClA,'k:','linewidth',1.5,'DisplayName','robot');hold on;
grid on;
ylabel('$C_L A\,[m^2]$','Interpreter','latex')
xlabel(angleName,'Interpreter','latex')
legend('Interpreter','latex','Location','best')
legend show

fig3 = figure();
plot(plotAngles,totalCsA,'k-','linewidth',1.5,'DisplayName','robot + support');hold on;
plot(plotAngles,robotCsA,'k:','linewidth',1.5,'DisplayName','robot');hold on;
grid on;
ylabel('$C_S A\,[m^2]$','Interpreter','latex')
xlabel(angleName,'Interpreter','latex')
legend('Interpreter','latex','Location','best')
legend show
