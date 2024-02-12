function [] = visualizeRobot(u, KinDynModel, Config)

    % VISUALIZEROBOT visualize the robot and collisions.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, Jun. 2022
    % Modified on Oct., 2022
    %

    % Set robot in the correct configuration
    iDynTreeWrappers.setRobotState(KinDynModel, Config.robot.w_H_b_init, u, zeros(6,1), zeros(Config.robot.ndof,1), Config.robot.gravityAcc);

    % Visualize the robot
    % robotFig = 
    iDynTreeWrappers.prepareVisualization(KinDynModel, Config.robot.meshesPath, 'color', [0.9,0.9,0.9], 'material', 'metal', ...
                                                     'transparency', 0.95, 'debug', true, 'view', [-92.9356 22.4635],...
                                                     'groundOn', false, 'groundColor', [0.5 0.5 0.5], 'groundTransparency', 0.5);

%     % Get figure number
%     figNum = robotFig.mainHandler.Number;
% 
%     % Visualize all collision spheres
%     for i = 1:length(Config.collisions.framesList)
%  
%         w_H_i      = iDynTreeWrappers.getWorldTransform(KinDynModel, Config.collisions.framesList{i}); 
%         centers_i  = computeSelfCollisionCenters(w_H_i, Config.collisions.(Config.collisions.framesList{i}).centers);
%         radiuses_i = Config.collisions.(Config.collisions.framesList{i}).radiuses;
% 
%         plotSelfCollisions(centers_i, radiuses_i, figNum)
%     end
end