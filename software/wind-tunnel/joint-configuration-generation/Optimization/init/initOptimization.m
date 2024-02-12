% INITOPTIMIZATION Initialization for the joints and jets position
%                  optimization. This file is for the robot iRonCub-Mk1.
%
% Author: Gabriele Nava (gabriele.nava@iit.it)
% Modified by Fabio Di Natale
% Genova: Oct. 2022
%

% Weights on controllability task
Config.opti.WeightCostFunction = eye(Config.robot.ndof);
                   
%--- Joints Positions Limits [min, max] ---%
Config.opti.upperBound = Config.robot.maxJointLimits*pi/180;
Config.opti.lowerBound = Config.robot.minJointLimits*pi/180;

%-------------------------------------------------------------------------%
% set options of the nonlinear optimization

% fmincon properties:
%                     Algorithm: 'interior-point'
%            BarrierParamUpdate: 'monotone'
%                CheckGradients: 0
%           ConstraintTolerance: 1.0000e-06
%                       Display: 'final'
%         EnableFeasibilityMode: 0
%      FiniteDifferenceStepSize: 'sqrt(eps)'
%          FiniteDifferenceType: 'forward'
%          HessianApproximation: 'bfgs'
%                    HessianFcn: []
%            HessianMultiplyFcn: []
%                   HonorBounds: 1
%        MaxFunctionEvaluations: 3000
%                 MaxIterations: 1000
%                ObjectiveLimit: -1.0000e+20
%           OptimalityTolerance:  1.0000e-06
%                     OutputFcn: []
%                       PlotFcn: []
%                  ScaleProblem: 0
%     SpecifyConstraintGradient: 0
%      SpecifyObjectiveGradient: 0
%                 StepTolerance: 1.0000e-10
%           SubproblemAlgorithm: 'factorization'
%                      TypicalX: 'ones(numberOfVariables,1)'
%                   UseParallel: 0
%
Config.opti.fminconOptions = optimoptions('fmincon', 'algorithm', 'interior-point','MaxFunEvals',7000,'MaxIter',7000,'SpecifyConstraintGradient',true, ...
                                          'ConstraintTolerance', 1e-6, 'OptimalityTolerance', 1e-6, 'StepTolerance', 1e-6, ...
                                          'Display', 'final-detailed', 'PlotFcn', {'optimplotfvalconstr','optimplotconstrviolation'}, ...
                                          'CheckGradients', false, 'ScaleProblem', false, 'FiniteDifferenceType', 'forward', 'SpecifyObjectiveGradient', true);
