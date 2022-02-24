%% configuration for the matlab iDyntree visualizer
% Different from Config since matlab.System can handle structures only if
% nontunable (Config in all the functions of the controller is used as
% tunable parameter).


component_path = getenv('IRONCUB_COMPONENT_SOURCE_DIR');

confVisualizer.robotName = 'iRonCub-Mk1_1';

confVisualizer.fileName = 'model.urdf';

confVisualizer.meshFilePrefix = [component_path '/models'];
confVisualizer.modelPath = [component_path '/models/' confVisualizer.robotName '/iRonCub/robots/' confVisualizer.robotName '/'];

confVisualizer.jointOrder = {'torso_pitch', 'torso_roll', 'torso_yaw', ...
                            'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow', ...
                            'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', ...
                            'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll', ...
                            'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};

confVisualizer.joints_positions = Config.initialConditions.joints;

confVisualizer.world_H_base = eye(4, 4);
confVisualizer.world_H_base(1:3, 1:3) = Config.initialConditions.orientation;
confVisualizer.world_H_base(1:3, 4) = Config.initialConditions.base_position;

confVisualizer.aroundRobot = 1; % square you see around the robot

confVisualizer.tStep = 0.01;%0.005;
