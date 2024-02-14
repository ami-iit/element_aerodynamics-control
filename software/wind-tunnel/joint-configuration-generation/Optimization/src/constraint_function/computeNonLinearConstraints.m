function [c, ceq, grad_c, grad_ceq] = computeNonLinearConstraints(u, KinDynModel, Config)

% COMPUTENONLINEARCONSTRAINTS computes the nonlinear constraints for
%                             joints position optimization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, Jun. 2022
% Modified on Oct., 2022
%

% update robot state for all constraints
jointPos = u;

w_H_b = Config.robot.w_H_b_init;

% set the current model state
iDynTreeWrappers.setRobotState(KinDynModel, w_H_b, jointPos, zeros(6,1), zeros(Config.robot.ndof,1), Config.robot.gravityAcc);

%---------------------------------------------------------------------%
% nonlinear equality constraints
ceq      = [];
grad_ceq = [];

% nonlinear inequality constraints

% add self-collision detection constraint
[c_collision, grad_c_collision] = computeSelfCollisionsAndGradient(KinDynModel, Config);

%---------------------------------------------------------------------%
% stack all constraints
c = c_collision;
grad_c = transpose(grad_c_collision);
end