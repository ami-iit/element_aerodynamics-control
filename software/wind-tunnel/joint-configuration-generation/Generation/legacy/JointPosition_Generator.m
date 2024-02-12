% This script generates different set of angles of the joints of the robot iRonCub.
% In particular, in this version, the angels changed are: Torso Roll, Torso Pitch, Shoulder Roll, Shoulder Pitch, Hip Roll and Hip Pitch.

clearvars
clc

%--- Joints Scale Factor ---%

Torso_ScaleFactor = 1;
Left_Shoulder_ScaleFactor = 1;
Right_Shoulder_ScaleFactor = 1;
Left_Hip_ScaleFactor = 1;
Right_Hip_ScaleFactor = 1;

%-- Joints Limits ---%

Torso_Joints_Limits = [-10,60;-15,15]; % [-20,20] % Order: Pitch, Roll, Yaw.
Left_Shoulder_Joints_Limits = [-50,-5;11,50]; % [1,50; 15,105] % Order: Pitch, Roll,Yaw, Elbow.
Right_Shoulder_Joints_Limits = [-50,-5;11,50]; % [1,50; 15,105] % Order: Pitch, Roll,Yaw, Elbow.
Left_Hip_Joints_Limits = [-20,40;5,30]; % [4,9; -60,0] % Order: Pitch, Roll, Yaw, Knee.
Right_Hip_Joints_Limits = [-20,40;5,30]; % [4,9; -60,0] % Order: Pitch, Roll, Yaw, Knee.
% Ankle_Joints_Limits = [-30,30;-20,20]; % Order Pitch, Roll.

%--- Samplings ---%

Samples = 10;

Torso_Roll_Samplings = Samples;
Torso_Pitch_Samplings = Samples;
Left_Shoulder_Roll_Samplings = Samples;
Left_Shoulder_Pitch_Samplings = Samples;
Right_Shoulder_Roll_Samplings = Samples;
Right_Shoulder_Pitch_Samplings = Samples;
Left_Hip_Roll_Samplings = Samples;
Left_Hip_Pitch_Samplings = Samples;
Right_Hip_Roll_Samplings = Samples;
Right_Hip_Pitch_Samplings = Samples;

%--- Number of Selected Positions ---%

Selected_Positions = 20;

Torso_Selected_Position = Selected_Positions;
Left_Shoulder_Selected_Position = Selected_Positions;
Right_Shoulder_Selected_Position = Selected_Positions;
Left_Hip_Selected_Position = Selected_Positions;
Right_Hip_Selected_Position = Selected_Positions;

%--- Coupled Joints Positions Generation ---%

Torso_Joints = Torso_Position_Function(Torso_ScaleFactor , Torso_Joints_Limits, Torso_Roll_Samplings, Torso_Pitch_Samplings, Torso_Selected_Position);
Left_Shoulder_Joints = Left_Shoulder_Position_Function(Left_Shoulder_ScaleFactor, Left_Shoulder_Joints_Limits, Left_Shoulder_Roll_Samplings, Left_Shoulder_Pitch_Samplings, Left_Shoulder_Selected_Position);
Right_Shoulder_Joints = Right_Shoulder_Position_Function(Right_Shoulder_ScaleFactor, Right_Shoulder_Joints_Limits, Right_Shoulder_Roll_Samplings, Right_Shoulder_Pitch_Samplings, Right_Shoulder_Selected_Position);
Left_Hip_Joints = Left_Hip_Position_Function(Left_Hip_ScaleFactor, Left_Hip_Joints_Limits, Left_Hip_Roll_Samplings, Left_Hip_Pitch_Samplings, Left_Hip_Selected_Position);
Right_Hip_Joints = Right_Hip_Position_Function(Right_Hip_ScaleFactor, Right_Hip_Joints_Limits, Right_Hip_Roll_Samplings, Right_Hip_Pitch_Samplings, Right_Hip_Selected_Position);

%--- Creation of Robot Configurations ---%

Random_Configuration_Torso = randperm(Selected_Positions);
Random_Configuration_Left_Shoulder = randperm(Selected_Positions);
Random_Configuration_Right_Shoulder = randperm(Selected_Positions);
Random_Configuration_Left_Hip = randperm(Selected_Positions);
Random_Configuration_Right_Hip = randperm(Selected_Positions);

Configuration_To_Generate = 15;

for i=1:Selected_Positions
    
    eval(['Robot_Configuration_0' num2str(i) '=[Torso_Joints(i,:), 0, Left_Shoulder_Joints(i,:), 40, 15, Right_Shoulder_Joints(i,:), 40, 15, Left_Hip_Joints(i,:), 7, 0, 0, 0, Right_Hip_Joints(i,:), 7, 0, 0, 0];'])
    
    Configuration_Matrix(i,:) = ["Robot_Configuration_0"+num2str(i) ,eval(['Robot_Configuration_0' num2str(i) '(1,:)'])];

end

writematrix(Configuration_Matrix,"Hovering_Configurations.csv");

% for i=Selected_Positions:Configuration_To_Generate
%     eval(['Robot_Configuration_0' num2str(i) '=[Torso_Joints(Random_Configuration_Torso(i),:), Left_Shoulder_Joints(Random_Configuration_Left_Shoulder(i),:), Right_Shoulder_Joints(Random_Configuration_Right_Shoulder(i),:), Left_Hip_Joints(Random_Configuration_Left_Hip(i),:), Right_Hip_Joints(Random_Configuration_Right_Hip(i),:)];'])
% end
