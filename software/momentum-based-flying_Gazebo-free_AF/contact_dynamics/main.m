clc, clear, close all

import iDynTree.*

icubModelsInstallPrefix = getenv('COMPONENT_IRONCUB_PREFIX');

meshFilePrefix = [ icubModelsInstallPrefix '/component_ironcub/models'];
% Select the robot using the folder name
robotName='iRonCub-Mk1';

modelPath = [icubModelsInstallPrefix '/component_ironcub/models/iRonCub-Mk1/iRonCub/robots/iRonCub-Mk1/'];
fileName='model_stl.urdf';

jointOrder = {'torso_pitch','torso_roll','torso_yaw',...
    'l_shoulder_pitch','l_shoulder_roll','l_shoulder_yaw','l_elbow', ...
    'r_shoulder_pitch','r_shoulder_roll','r_shoulder_yaw','r_elbow', ...
    'l_hip_pitch','l_hip_roll','l_hip_yaw','l_knee','l_ankle_pitch','l_ankle_roll', ...
    'r_hip_pitch','r_hip_roll','r_hip_yaw','r_knee','r_ankle_pitch','r_ankle_roll'};

% Main variable of iDyntreeWrappers used for many things including updating
% robot position and getting world to frame transforms
KinDynModel = iDynTreeWrappers.loadReducedModel(jointOrder,'root_link',modelPath,fileName,false);

% create vector of positions

joints_positions = [0.4744; 0.0007; 0.0001; ...
    -0.1745; 0.4363; 0.6981; 0.2618; ...
    -0.1745; 0.4363; 0.6981; 0.2618; ...
    0.9; 0.0000; -0.0001; -0.4; -0.0004; 0.0003; ...
    0.0003; 0.0001; -0.0002; -0.0004; -0.0005; 0.0003];
% add a world to base mainly to avoid overlap of coordinate frame and robot
world_H_base=[1,0,0,0;...
    0,1,0,0;...
    0,0,1,0.61;...
    0,0,0,1];

Config.initialConditions.base_position = [0;0;0.63];
Config.initialConditions.orientation = diag([-1,-1,1]);

gravity_vector = [0;0;-9.81];

% Set initial position of the robot
iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),gravity_vector);

% Prepare figure, handles and variables required for the update, some extra
% options are commented.
[visualizer,objects]=iDynTreeWrappers.prepareVisualization(KinDynModel,meshFilePrefix,...
    'color',[1,1,1],'material','metal','transparency',1,'debug',true,'view',[-160.9356   22.4635],...
    'groundOn',true,'groundColor',[0.5 0.5 0.5], 'groundTransparency',0.5);%,... % optional inputs [-92.9356   22.4635]

iDynTreeWrappers.setRobotState(KinDynModel,world_H_base,joints_positions,zeros(6,1),zeros(size(joints_positions)),gravity_vector);


left_force = line([0,0], [0,0], [0,0], ...                   % Output 'Arrow'
    'linewidth', 5, ...
    'color', 'r');

right_force = line([0,0], [0,0], [0,0], ...                   % Output 'Arrow'
    'linewidth', 5, ...
    'color', 'r');


iDynTreeWrappers.updateVisualization(KinDynModel,visualizer); hold on

a = (KinDynModel.kinDynComp.getCenterOfMassPosition());
b = (KinDynModel.kinDynComp.getWorldTransform('l_arm_jet_turbine'));

turbineList   = {'chest_l_jet_turbine','chest_r_jet_turbine','l_arm_jet_turbine','r_arm_jet_turbine'};

H = iDynTreeWrappers.getWorldTransform(KinDynModel,turbineList{3});

% Robot frames list
Frames.BASE_LINK        = 'root_link';
Frames.JET1_FRAME       = 'l_arm_jet_turbine';
Frames.JET2_FRAME       = 'r_arm_jet_turbine';
Frames.JET3_FRAME       = 'chest_l_jet_turbine';
Frames.JET4_FRAME       = 'chest_r_jet_turbine';
Frames.COM_FRAME        = 'com';
Frames.LFOOT_FRAME      = 'l_sole';
Frames.RFOOT_FRAME      = 'r_sole';

free_floating_generalized_torque = iDynTree.FreeFloatingGeneralizedTorques(KinDynModel.kinDynComp.model);

ack = KinDynModel.kinDynComp.generalizedBiasForces(free_floating_generalized_torque);

% convert to Matlab format: compute the base bias acc (h_b) and the
% joint bias acc (h_s) and concatenate them
h_b = free_floating_generalized_torque.baseWrench.toMatlab;
h_s = free_floating_generalized_torque.jointTorques.toMatlab;
bias_forces = [h_b;h_s];

mass_matrix_iDyn = iDynTree.MatrixDynSize();

KinDynModel.kinDynComp.getFreeFloatingMassMatrix(mass_matrix_iDyn);

mass_matrix = mass_matrix_iDyn.toMatlab;

JLeft_Jac_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF+6);
JRight_Jac_iDyntree = iDynTree.MatrixDynSize(6,KinDynModel.NDOF+6);

JDot_nu_LFOOT_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(Frames.LFOOT_FRAME);
JDot_nu_LFOOT = JDot_nu_LFOOT_iDyntree.toMatlab;
JDot_nu_RFOOT_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(Frames.RFOOT_FRAME);
JDot_nu_RFOOT = JDot_nu_RFOOT_iDyntree.toMatlab;
JDot_nu_feet = [JDot_nu_LFOOT; JDot_nu_RFOOT];

LFOOT_frameID = KinDynModel.kinDynComp.getFrameIndex(Frames.LFOOT_FRAME);
RFOOT_frameID = KinDynModel.kinDynComp.getFrameIndex(Frames.RFOOT_FRAME);

BASE_frameID = KinDynModel.kinDynComp.getFrameIndex(Frames.BASE_LINK);

ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(LFOOT_frameID, JLeft_Jac_iDyntree);
ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(RFOOT_frameID, JRight_Jac_iDyntree);

% check for errors
if ~ack
    error('[getRelativeJacobian]: unable to get the relative jacobian from the reduced model.')
end

% covert to Matlab format
JLeft_frameJac = JLeft_Jac_iDyntree.toMatlab;
JRight_frameJac = JRight_Jac_iDyntree.toMatlab;

J_feet = [JLeft_frameJac; JRight_frameJac];

% M_inv = inv(mass_matrix + eye(size(mass_matrix,1)*0.1))

G = J_feet*(mass_matrix\J_feet');

% k = J_feet*(mass_matrix\( -bias_forces));
% G_inv = eye(size(G,1))\G
% G_inv1 = inv(G)

% contact_wrench = -G\(J_feet*(mass_matrix\(-bias_forces)));
init_point = [0, 0, 20, 0, 0, 0, 0, 0, 20, 0, 0, 0]';
H_LFOOT = iDynTreeWrappers.getWorldTransform(KinDynModel,Frames.LFOOT_FRAME);
H_RFOOT = iDynTreeWrappers.getWorldTransform(KinDynModel,Frames.RFOOT_FRAME);
left_z_pos = H_LFOOT(3,4);
right_z_pos = H_RFOOT(3,4);
contact_point = [left_z_pos; right_z_pos];
contact_wrench = compute_unilateral_contact(J_feet, mass_matrix, zeros(23,1), bias_forces, JDot_nu_feet, contact_point, init_point);

% fc1 = -inv(G)*(J_feet*inv(mass_matrix)*(-bias_forces))
% fc = -G\k
% fc1 = -G_inv*k
% fc2 =  - eye(size(G,1))\(J_feet*(mass_matrix\J_feet'))*J_feet*(mass_matrix\( -bias_forces))

joint_des_pos = joints_positions;
joint_velocity = zeros(23,1);

q_at_k.p = world_H_base(1:3, 4);
q_at_k.R = world_H_base(1:3, 1:3);
q_at_k.s = joints_positions;

qdot_at_k.p = zeros(3,1);
qdot_at_k.R = zeros(3,1);
qdot_at_k.s = zeros(23,1);

left_in_contact = 0;
right_in_contact = 0;

pause

for i=1:1000
    
    disp('iteration')
    disp(i)
    torque = compute_torque(q_at_k.s, joint_des_pos, qdot_at_k.s, 0.1);
    
    ack = KinDynModel.kinDynComp.generalizedBiasForces(free_floating_generalized_torque);
    ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(LFOOT_frameID, JLeft_Jac_iDyntree);
    
    
    %     ack = KinDynModel.kinDynComp.getRelativeJacobian(LFOOT_frameID, BASE_frameID, JLeft_Jac_iDyntree)
    
    
    ack = KinDynModel.kinDynComp.getFrameFreeFloatingJacobian(RFOOT_frameID, JRight_Jac_iDyntree);
    ack = KinDynModel.kinDynComp.getFreeFloatingMassMatrix(mass_matrix_iDyn);
    
    h_b = free_floating_generalized_torque.baseWrench.toMatlab;
    h_s = free_floating_generalized_torque.jointTorques.toMatlab;
    bias_forces = [h_b;h_s];
    
    JLeft_frameJac = JLeft_Jac_iDyntree.toMatlab;
    JRight_frameJac = JRight_Jac_iDyntree.toMatlab;
    
    JDot_nu_LFOOT_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(Frames.LFOOT_FRAME);
    JDot_nu_LFOOT = JDot_nu_LFOOT_iDyntree.toMatlab;
    JDot_nu_RFOOT_iDyntree = KinDynModel.kinDynComp.getFrameBiasAcc(Frames.RFOOT_FRAME);
    JDot_nu_RFOOT = JDot_nu_RFOOT_iDyntree.toMatlab;
    JDot_nu_feet = [JDot_nu_LFOOT; JDot_nu_RFOOT];
    
    mass_matrix = mass_matrix_iDyn.toMatlab;
    
    H_LFOOT = iDynTreeWrappers.getWorldTransform(KinDynModel,Frames.LFOOT_FRAME);
    H_RFOOT = iDynTreeWrappers.getWorldTransform(KinDynModel,Frames.RFOOT_FRAME);
    
    JLeft_frameJac_ver = [JLeft_frameJac(1:3,:) - wbc.skew(H_LFOOT(1:3,1:3)*[-0.07;-0.045;0])*JLeft_frameJac(4:6,:); zeros(3,29)];
    
    J_feet = [JLeft_frameJac; JRight_frameJac];
    
    G = J_feet*(mass_matrix\J_feet');
    
    left_z_pos = H_LFOOT(3,4);
    right_z_pos = H_RFOOT(3,4);
    contact_point = [left_z_pos; right_z_pos]
    left_in_contact(end+1) = left_z_pos <= 0;
    right_in_contact(end+1) = right_z_pos <= 0;
    
    %     contact_wrench = -G\(-J_feet*[q_ddot.p; q_ddot.R; q_ddot.s] + J_feet*(mass_matrix\(S*torque - bias_forces)))
    %     contact_wrench = compute_contact_wrench(J_feet, JDot_nu_feet, mass_matrix, torque, bias_forces, contact_point)
    contact_wrench = compute_unilateral_contact(J_feet, mass_matrix, torque, bias_forces, JDot_nu_feet, contact_point, contact_wrench)
    
    %     contact_wrench = [zeros(12,1)];
    %     contact_wrench(3) = 300;
    %     contact_wrench(3+6) = 300;
    
    
    
    generalized_external_wrenches = J_feet'*contact_wrench;
    
    q_ddot = forward_dynamics(mass_matrix, bias_forces, torque, generalized_external_wrenches);
    
    acc_RFoot(i, :) = JDot_nu_feet + J_feet*[q_ddot.p; q_ddot.R; q_ddot.s];
    
    %     [q, q_dot] = integrate_configuration(q_at_k, qdot_at_k, q_ddot);
    
    [q, q_dot] = integrate_configuration(q_at_k, qdot_at_k, q_ddot);
    
    if left_in_contact(end) == 1 && left_in_contact(end-1) == 0
        N = (eye(29) - inv(mass_matrix)*J_feet'*inv(J_feet*inv(mass_matrix)*J_feet')*J_feet);
        x = N*[q_dot.p; q_dot.R; q_dot.s];
        q_dot.p = x(1:3);
        q_dot.R = x(4:6);
        q_dot.s = x(7:end);
        disp('Conctact detected Left')
    end
    
    if right_in_contact(end) == 1 && right_in_contact(end-1) == 0
        N = (eye(29) - inv(mass_matrix)*J_feet'*inv(J_feet*inv(mass_matrix)*J_feet')*J_feet);
        x = N*[q_dot.p; q_dot.R; q_dot.s];
        q_dot.p = x(1:3);
        q_dot.R = x(4:6);
        q_dot.s = x(7:end);
        disp('Conctact detected Right')
    end
    
    iDynTreeWrappers.setRobotState(KinDynModel, q.H, q.s, [q_dot.p; q_dot.R], q_dot.s, gravity_vector);
    
    set(right_force, "XData", [H_RFOOT(1,4), H_RFOOT(1,4)+ 0.003*contact_wrench(7)],...
        "YData", [H_RFOOT(2,4), H_RFOOT(2,4)+ 0.003*contact_wrench(8)],...
        "ZData", [H_RFOOT(3,4), H_RFOOT(3,4)+ 0.003*contact_wrench(9)]);
    
    set(left_force, "XData", [H_LFOOT(1,4), H_LFOOT(1,4)+ 0.003*contact_wrench(1)],...
        "YData", [H_LFOOT(2,4), H_LFOOT(2,4)+ 0.003*contact_wrench(2)],...
        "ZData", [H_LFOOT(3,4), H_LFOOT(3,4)+ 0.003*contact_wrench(3)]);
    
    drawnow
    
    iDynTreeWrappers.updateVisualization(KinDynModel,visualizer);
    
    %     hold on
    %     if i ~= 1, delete(Force); end
    %     Force = arrow(H_RFOOT(1:3,4), H_RFOOT(1:3,4) + 0.005*contact_wrench(7:9));
    
    
    
    
    
    q_at_k = q;
    qdot_at_k = q_dot;
    
    x(i) = q.p(1);
    y(i) = q.p(2);
    z(i) = q.p(3);
        generate_gif(i, gcf)
    %     pause;
end

function torque = compute_torque(joint_pos, joint_des_pos, joint_vel, K)
Kp = K;
Kv = 2*sqrt(Kp);
torque = -Kp*(joint_pos - joint_des_pos) - Kv*joint_vel;
end

function q_ddot =  forward_dynamics(mass_matrix, bias_forces, torques, generalized_external_wrenches)
S = [zeros(6,23); eye(23)];
ddot = mass_matrix\(S*torques + generalized_external_wrenches - bias_forces);
q_ddot.p = ddot(1:3);
q_ddot.R = ddot(4:6);
q_ddot.s = ddot(7:end);
end

function [q, q_dot] = integrate_configuration(q_at_k, qdot_at_k, q_ddot)
dt = 0.001;
q.p = q_at_k.p + qdot_at_k.p*dt + q_ddot.p*dt^2/2;
disp(q.p)
% q.R = q_at_k.R*exp(wbc.skew(qdot_at_k.R)*dt);

gain = 0.001;
A = gain*((q_at_k.R'*q_at_k.R)' - eye(3));
R_dot = (wbc.skew(qdot_at_k.R) + A)*q_at_k.R;
% R_dot = q_at_k.R*wbc.skew(qdot_at_k.R)*dt;
q.R = q_at_k.R + R_dot*dt;
q.s = q_at_k.s + qdot_at_k.s*dt; % + q_ddot.s*dt*2/2;
q.H = eye(4);
q.H(1:3,1:3) = q.R;
q.H(1:3,4)   = q.p;

q_dot.p = qdot_at_k.p + q_ddot.p*dt;
q_dot.R = qdot_at_k.R + q_ddot.R*dt;
q_dot.s = qdot_at_k.s + q_ddot.s*dt;
end

function [q, q_dot] = integrate_with_ode(q_at_k, qdot_at_k, q_ddot)
dt = 0.002;

[~, y] = ode15s(@(t,y) qdot_at_k.p, [0 dt], q_at_k.p);
q.p = y(end,:)';

gain = 0.001;
A = gain*((q_at_k.R'*q_at_k.R)' - eye(3));
% R_dot = @(t) (wbc.skew(qdot_at_k.R) + A)*q_at_k.R;
% q.R = integral(R_dot, 0, dt, 'ArrayValued', true) + q_at_k.R;

R_dot = (wbc.skew(qdot_at_k.R) + A)*q_at_k.R;

% [~, y] = ode15s(@(t,y) R_dot(1:3,1), [0 dt], q_at_k.R(1:3,1));
% q.R(1:3,1) = y(end,:)';
%
% [~, y] = ode15s(@(t,y) R_dot(1:3,2), [0 dt], q_at_k.R(1:3,2));
% q.R(1:3,2) = y(end,:)';
%
% [~, y] = ode15s(@(t,y) R_dot(1:3,3), [0 dt], q_at_k.R(1:3,3));
% q.R(1:3,3) = y(end,:)';


q.R = q_at_k.R*expm(wbc.skew(qdot_at_k.R*dt))

[~, y] = ode15s(@(t,y) qdot_at_k.s, [0 dt], q_at_k.s);
q.s = y(end,:)';

[~, y] = ode15s(@(t,y) q_ddot.p, [0 dt], qdot_at_k.p);
q_dot.p = y(end,:)';

[~, y] = ode15s(@(t,y) q_ddot.R, [0 dt], qdot_at_k.R);
q_dot.R = y(end,:)';

[~, y] = ode15s(@(t,y) q_ddot.s, [0 dt], qdot_at_k.s);
q_dot.s = y(end,:)';


q.H = eye(4);
q.H(1:3,1:3) = q.R;
q.H(1:3,4)   = q.p;

end

function contact_wrench = compute_contact_wrench(J_feet, JDot_nu_feet, mass_matrix, torque, bias_forces, contact_point)
G = J_feet*(mass_matrix\J_feet');
S = [zeros(6,23); eye(23)];
contact_wrench = -G\(JDot_nu_feet + J_feet*(mass_matrix\(S*torque - bias_forces)));
contact_wrench(1:6) = contact_wrench(1:6)*(contact_point(1)<=0);
contact_wrench(7:12) = contact_wrench(7:12)*(contact_point(2)<=0);
end

function contact_wrench = compute_contact_wrench2(J_feet, JDot_nu_feet, mass_matrix, torque, bias_forces, contact_point)

contact_wrench = zeros(12,1);
S = [zeros(6,23); eye(23)];
if contact_point(1) <= 0
    G = [mass_matrix   , -J_feet(1:6,:)';...
        J_feet(1:6,:) , zeros(6)];
    b = [S*torque - bias_forces;...
        JDot_nu_feet(1:6,:)];
    x = G\b;
    contact_wrench(1:6) = x(30 :end);
end

if contact_point(2) <= 0
    G = [mass_matrix   , -J_feet(7:12,:)';...
        J_feet(7:12,:) , zeros(6)];
    b = [S*torque - bias_forces;...
        JDot_nu_feet(7:12,:)];
    x = G\b;
    contact_wrench(7:12) = x(30 :end);
end

if contact_point(1) <= 0 && contact_point(2) <= 0
    G = [mass_matrix, -J_feet';...
        J_feet     , zeros(12)];
    b = [S*torque - bias_forces;...
        JDot_nu_feet];
    x = G\b;
    contact_wrench = x(30   :end);
end
end

function free_acceleration = compute_free_acceleration(mass_matrix, torque, bias_forces)
S = [zeros(6,23); eye(23)];
free_acceleration = mass_matrix\(S*torque - bias_forces);
end

function free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet)
free_contact_acceleration = J_feet*free_acceleration + JDot_nu_feet;
end

function wrench  = compute_unilateral_contact(J_feet, mass_matrix, torque, bias_forces, JDot_nu_feet, contact_point, init_point)
free_acceleration = compute_free_acceleration(mass_matrix, torque, bias_forces);
free_contact_acceleration = compute_free_contact_acceleration(J_feet, free_acceleration, JDot_nu_feet);
G = J_feet*(mass_matrix\J_feet');
mu = 0.1;
options = optimoptions('fmincon','Algorithm','interior-point'); % run interior-point algorithm
% init_point = [0, 0, 20, 0, 0, 0, 0, 0, 20, 0, 0, 0]';
wrench = fmincon(@(x) cost(x,G, free_contact_acceleration),init_point,[],[],[],[],[],[],@(x) constr(x, mu, contact_point),options);
end

function c = cost(x, G, free_contact_acceleration)
c = 0.5*x'*G*x + x'*free_contact_acceleration;
end

function [c, ceq] = constr(x, mu, contact_point)
c = [sqrt(x(1)^2 + x(2)^2) - mu*x(3)^2;...
    sqrt(x(1+6)^2 + x(2+6)^2) - mu*x(3+6)^2;...
    -x(3);...
    -x(3+6);];
ceq = [x(3)*(contact_point(1)>0);...
    x(3+6)*(contact_point(2)>0)];
% ceq = [];
end