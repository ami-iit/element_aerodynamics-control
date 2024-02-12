% This function generates different set of angles for the torso joints of the robot iRonCub.

function Left_Shoulder_Joints = Left_Shoulder_Position_Function(Scale_Factor, Joints_Limits, Samplings_Roll, Samplings_Pitch, Selected_Positions)

%--- Joints Positions Limits Scale Factor ---%

scaleShoulderJointsScaleFactor = Scale_Factor;

%--- Joints Positions Limits Scale Factor ---%

shoulderJointsLimits = scaleShoulderJointsScaleFactor * Joints_Limits;

%--- Modification Parameters ---%

Number_of_Sampling_Roll = Samplings_Roll; %Number of samplings for roll.
Number_of_Sampling_Pitch = Samplings_Pitch; %Number of samplings for pitch.

% Update the robot joint position according to the modification to be applied.

Left_Shoulder_Pitch =  linspace(shoulderJointsLimits(1), shoulderJointsLimits(2), Number_of_Sampling_Pitch);
Left_Shoulder_Roll = linspace(shoulderJointsLimits(3), shoulderJointsLimits(4), Number_of_Sampling_Roll); 

round(Left_Shoulder_Roll);
round(Left_Shoulder_Pitch);

[Left_Shoulder_Roll, Left_Shoulder_Pitch] = meshgrid(Left_Shoulder_Roll, Left_Shoulder_Pitch);

Left_Shoulder_Roll  = reshape(Left_Shoulder_Roll,[],1);
Left_Shoulder_Pitch = reshape(Left_Shoulder_Pitch,[],1);

%scatter(Left_Shoulder_Roll,Left_Shoulder_Pitch)

New_Left_Shoulder_Joints(:,1) = round(Left_Shoulder_Roll,2);
New_Left_Shoulder_Joints(:,2) = round(Left_Shoulder_Pitch,2);

figure(2);
plot(New_Left_Shoulder_Joints(:,1), New_Left_Shoulder_Joints(:,2),'.');
title("Left Shoulder")
Left_Shoulder_Joints = zeros(Selected_Positions,2);

Randomly_Selected_Configurations = randperm(Samplings_Roll*Samplings_Pitch);

for i=1:length(Left_Shoulder_Joints)
    Left_Shoulder_Joints(i,1)= New_Left_Shoulder_Joints(Randomly_Selected_Configurations(i),1);
    Left_Shoulder_Joints(i,2)= New_Left_Shoulder_Joints(Randomly_Selected_Configurations(i),2);
end

hold on
plot(Left_Shoulder_Joints(:,1), Left_Shoulder_Joints(:,2), 'or');
hold off

end