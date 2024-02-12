% This function generates different set of angles for the torso joints of the robot iRonCub.

function Left_Hip_Joints = Left_Hip_Position_Function(Scale_Factor, Joints_Limits, Samplings_Roll, Samplings_Pitch, Selected_Positions)

%--- Joints Positions Limits Scale Factor ---%

scaleHipJointsScaleFactor = Scale_Factor;

%--- Joints Positions Limits Scale Factor ---%

hipJointsLimits = scaleHipJointsScaleFactor * Joints_Limits;

%--- Modification Parameters ---%

Number_of_Sampling_Roll = Samplings_Roll; %Number of samplings for roll.
Number_of_Sampling_Pitch = Samplings_Pitch; %Number of samplings for pitch.

% Update the robot joint position according to the modification to be applied.

Left_Hip_Pitch =  linspace(hipJointsLimits(1), hipJointsLimits(2), Number_of_Sampling_Pitch);
Left_Hip_Roll = linspace(hipJointsLimits(3), hipJointsLimits(4), Number_of_Sampling_Roll); 

round(Left_Hip_Roll);
round(Left_Hip_Pitch);

[Left_Hip_Roll, Left_Hip_Pitch] = meshgrid(Left_Hip_Roll, Left_Hip_Pitch);

Left_Hip_Roll  = reshape(Left_Hip_Roll,[],1);
Left_Hip_Pitch = reshape(Left_Hip_Pitch,[],1);

%scatter(Left_Hip_Roll,Left_Hip_Pitch)

New_Left_Hip_Joints(:,1) = round(Left_Hip_Roll,2);
New_Left_Hip_Joints(:,2) = round(Left_Hip_Pitch,2);

figure(4);
plot(New_Left_Hip_Joints(:,1), New_Left_Hip_Joints(:,2),'.');
title("Left Hip")
Left_Hip_Joints = zeros(Selected_Positions,2);

Randomly_Selected_Configurations = randperm(Samplings_Roll*Samplings_Pitch);

for i=1:length(Left_Hip_Joints)    
    Left_Hip_Joints(i,1)= New_Left_Hip_Joints(Randomly_Selected_Configurations(i),1);
    Left_Hip_Joints(i,2)= New_Left_Hip_Joints(Randomly_Selected_Configurations(i),2);
end

hold on
plot(Left_Hip_Joints(:,1), Left_Hip_Joints(:,2), 'or');
hold off

end