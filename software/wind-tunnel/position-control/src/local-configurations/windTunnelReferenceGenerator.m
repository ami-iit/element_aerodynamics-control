function  [jointPos_des] = ...
    windTunnelReferenceGenerator(jointPos_0, currentState, Config, time)

% WINDTUNNELREFERENCEGENERATOR: generates the references for performing
%                               the wind tunnel experiments with
%                               iRonCub.

%% --- Initialization ---

if isempty(currentState)
    % initialize currentState to 1 -> starting position
    currentState = -1;
end

% initialize joints reference positions output
jointPos_des = jointPos_0;

%% STATE CHANGE
if currentState > -1 && currentState <= (Config.N_joints_reference -1) && time > Config.initialBalancingTime
    % Set the joints reference positions to the selected one
    jointPos_des = Config.joints_references((currentState + 1),:)';
end

end