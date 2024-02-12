% This script generates in a random way the angles of the arms and legs joints of the robot iRonCub.

clear all
clc

%--- Joints Positions Limits Scale Factor ---%

scaleArmsJointsLimits  = 1;
scaleLegsJointsLimits  = 0.75;

%--- Joints Positions Limits Scale Factor ---%

armsJointsLimit        = scaleArmsJointsLimits * [-90, 10; 0, 160; -30, 80; 15-0.1, 100];
legsJointsLimit        = scaleLegsJointsLimits * [-35, 80; 5, 90; -70, 70; -35, 0+0.5; -30, 30; -20, 20];

%--- Joints Default Positions ---%

Default_Elbow_Position = 10; % Fixed Value can be changed with a variable. TO BE DISCUSSED.

%--- Joints Modifications ---%

Joints_List = [l_elbow];      % List of the Joints to be modified.

Max_Negative_Variation = -10; % Lower Limit of Possible Modification.
Max_Positive_Variation = 10;  % Upper Limit of Possible Modification.

% Generation of random angles for each joint.

rng('default');                          % Default Setting for random number generation.
random_modification = round(randn,2)*10; % First Attempt of Generating Random Modification.

% Verify that the modifications are in the specified range of modifications and if they aren't re-run the random generation.

Range_Acceptability_Check = False;

while Range_Acceptability_Check = False

	if (random_modification >= Max_Negative_Variation && random_modification <= Max_Positive_Variation)
		if (abs(random_modification) < 1)
			printf("The modification was in the specified range, but its absolute value was less then 1°. It was necessary to adjust it.")
			random_modification = random_modification*10;
			Green_Flag = True;
		else
			printf("The modification was in the specified range and no adjustment to its absolute value was necessary.")
			Range_Acceptability_Check = True;
		end
	else
		printf("The modification was not in the specified range. It was necessary to re-run the random generation.")
		Range_Acceptability_Check = False;
		random_modification = round(randn,2)*10;
	end
end 


% Update the robot joint position according to the modification to be applied.

Old_Configuration = Default_Elbow_Position;
New_Configuration = Old_Configuration + random_modification;

% Understand if the configuration is acceptable:
% 1. The new joints positions are in the allowed ranges;
% 2. There are no interferences with other parts of the robot.

% First Check

if (New_Configuration >= armsJointsLimit(4,1) && New_Configuration <= armsJointsLimit(4,2))
	printf("The new joint position is in the specified range.")
else	    
	if (New_Configuration < armsJointsLimit(4,1))
		New_configuration = armsJointsLimit(4,1);
		printf("The new joint position WAS LOWER than the minimum accepted value, thus the latter was set as new joint position.")
	else
		New_configuration = armsJointsLimit(4,1);
		printf("The new joint position WAS HIGHER than the maximum accepted value, thus the latter was set as new joint position.")
	end
end

%Second Check

%%%% TO BE DEVELOPED %%%%

% Write the acceptable configuration in a file.

file_ID = fopen('Joints_Position.txt','w');                       % Opening the file.

fprintf(file_ID,'%12s %12s\n','Joint','Angle');                   % Title of the columns.
fprintf(fileID,'%12s %12.2f\n', Joints_List, New_Configuration);  % Content of the columns.

fclose(fileID);                                                   % Closing the file.