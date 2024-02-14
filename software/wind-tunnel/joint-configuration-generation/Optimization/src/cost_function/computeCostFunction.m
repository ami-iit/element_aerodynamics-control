function [cost, grad_cost] = computeCostFunction(u, KinDynModel, Config)

    % COMPUTECOSTFUNCTION computes the cost function for joints position optimization.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    % Modified on OCt., 2022
    %

    % update robot state for all costs
    jointPos = u;

    w_H_b = Config.robot.w_H_b_init;

    % set the current model state
    iDynTreeWrappers.setRobotState(KinDynModel, w_H_b, jointPos, zeros(6,1), zeros(Config.robot.ndof,1), Config.robot.gravityAcc);

    %---------------------------------------------------------------------%
    
    cost = (u-Config.opti.u_des)'*Config.opti.WeightCostFunction*(u-Config.opti.u_des);
    grad_cost = 2*(u-Config.opti.u_des)'*Config.opti.WeightCostFunction;

end