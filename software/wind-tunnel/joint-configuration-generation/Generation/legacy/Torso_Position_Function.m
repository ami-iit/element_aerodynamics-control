% This function generates different set of angles for the torso joints of the robot iRonCub.

function Torso_Joints = Torso_Position_Function(Scale_Factor, Joints_Limits, Samplings_Roll, Samplings_Pitch, Selected_Positions)

%--- Joints Positions Limits Scale Factor ---%

scaleTorsoJointsScaleFactor = Scale_Factor;

%--- Joints Positions Limits Scale Factor ---%

torsoJointsLimits = scaleTorsoJointsScaleFactor * Joints_Limits;

%--- Modification Parameters ---%

Number_of_Sampling_Pitch = Samplings_Pitch; %Number of samplings for pitch.
Number_of_Sampling_Roll = Samplings_Roll; %Number of samplings for roll.

% Update the robot joint position according to the modification to be applied.

Torso_Pitch =  linspace(torsoJointsLimits(1), torsoJointsLimits(2), Number_of_Sampling_Pitch);
Torso_Roll = linspace(torsoJointsLimits(3), torsoJointsLimits(4), Number_of_Sampling_Roll); 

round(Torso_Roll);
round(Torso_Pitch);

[Torso_Roll, Torso_Pitch] = meshgrid(Torso_Roll, Torso_Pitch);

Torso_Roll  = reshape(Torso_Roll,[],1);
Torso_Pitch = reshape(Torso_Pitch,[],1);

%scatter(Torso_Roll,Torso_Pitch);

New_Torso_Joints(:,1) = round(Torso_Roll,2);
New_Torso_Joints(:,2) = round(Torso_Pitch,2);

figure(1);
plot(New_Torso_Joints(:,1), New_Torso_Joints(:,2),'.');
title("Torso")
Torso_Joints = zeros(Selected_Positions,2);

Randomly_Selected_Configurations = randperm(Samplings_Roll*Samplings_Pitch);

for i=1:length(Torso_Joints)
    Torso_Joints(i,1)= New_Torso_Joints(Randomly_Selected_Configurations(i),1);
    Torso_Joints(i,2)= New_Torso_Joints(Randomly_Selected_Configurations(i),2);
end

hold on
plot(Torso_Joints(:,1), Torso_Joints(:,2), 'or');
hold off

end