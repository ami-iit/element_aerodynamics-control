classdef RobotVisualizer < matlab.System & matlab.system.mixin.CustomIcon
    % matlab.System handling the robot visualization
    % go in app/robots/iRonCub*/initVisualizer.m to change the setup config

    %@author: Giuseppe L'Erario

    
    
    properties (Nontunable)
        config
       
        
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        KinDynModel, visualizer;
        g   = [0, 0, -9.81]; % gravity vector
        pov = [92.9356 22.4635]; % view vector for the visualizer
        jetFrameList = {'chest_l_jet_turbine', 'chest_r_jet_turbine', 'l_arm_jet_turbine', 'r_arm_jet_turbine'};
        X, Y, Z; % coordinate for the jet cones
        jets;
        
        %modification for plotting aerodynamics forces
        linkFrame= {'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'};
        
        aerodynamics_force_vector  ;
       
        X0,Y0,Z0; %coordinate for initial point of aerodynamics force vector
        XA,YA,ZA; %coordinate for end point of aerodynamics force vector
    end

    methods (Access = protected)

        function setupImpl(obj)
            

            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()  % get obj.visualizer

                if obj.config.visualizeJets
                    obj.prepareJets();
                end
                
                
                %prepare the initial position of aerodynamics force vector
                if obj.config.aerodynamics_forces
                    obj.prepareAerodynamics_forces();
                end
            end
           
           
            
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end

        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity, jetIntensities,aerodynamics_forces_wb)


           
            if obj.config.visualizeRobot
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                iDynTreeWrappers.updateVisualization(obj.KinDynModel, obj.visualizer);
                obj.followTheRobot();

                if obj.config.visualizeJets
                    obj.updateJets(jetIntensities);
                end
           
                if obj.config.aerodynamics_forces
                    obj.updateAerodynamics_forces(aerodynamics_forces_wb);
                end
            
            end
            
 
            
           
            
            
        end
        
       
            
        
        
            
        function prepareRobot(obj)
            % Main variable of iDyntreeWrappers used for many things including updating
            % robot position and getting world to frame transforms
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.config.jointOrder, 'root_link', ...
                obj.config.modelPath, obj.config.fileName, false);

            % Set initial position of the robot
            initial_base_velocity = zeros(6, 1);
            initial_joints_velocity = zeros(length(obj.config.joints_positions));

            iDynTreeWrappers.setRobotState(obj.KinDynModel, obj.config.world_H_base, obj.config.joints_positions, ...
                initial_base_velocity, initial_joints_velocity, obj.g);

            % Prepare figure, handles and variables required for the update, some extra
            % options are commented.
            [obj.visualizer, ~] = iDynTreeWrappers.prepareVisualization(obj.KinDynModel, obj.config.meshFilePrefix, ...
                'color', [1, 1, 1], 'material', 'metal', 'transparency', 0.8, 'debug', true, 'view', obj.pov, ...
                'groundOn', true, 'groundColor', [0.5 0.5 0.5], 'groundTransparency', 0.5);
            % The size of the visualizer matlab figure
            x0 = 300;
            y0 = 300;
            width = 1300;
            height = 1300;
            set(gcf, 'position', [x0, y0, width, height]);
            
            
            

            
            %             set(gcf,'doublebuffer','off');
        end

        function followTheRobot(obj)
            % moves the window around the robot
            comPosition = toMatlab(obj.KinDynModel.kinDynComp.getCenterOfMassPosition());
            xlim([comPosition(1) - obj.config.aroundRobot, comPosition(1) + obj.config.aroundRobot]);
            ylim([comPosition(2) - obj.config.aroundRobot, comPosition(2) + obj.config.aroundRobot]);
            zlim([comPosition(3) - obj.config.aroundRobot, comPosition(3) + obj.config.aroundRobot]);
        end

        function updateJets(obj, jetIntensities)
            % for every jet
            for i = 1:length(obj.jetFrameList)
                H = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, obj.jetFrameList{i});
                % resize the length of the cylinder wrt the intensity of
                % the thrust
                Z = obj.Z{i} * jetIntensities(i) / 1000;
                R = H(1:3, 1:3);
                % rotate the vectors
                v1 = R * [obj.X{i}(1, :); obj.Y{i}(1, :); Z(1, :)];
                v2 = R * [obj.X{i}(2, :); obj.Y{i}(2, :); Z(2, :)];
                % update the data in the jet surfaces (H(i,4) is the translation)
                set(obj.jets{i}, 'XData', [v1(1, :); v2(1, :)] + H(1, 4), ...
                    'YData', [v1(2, :); v2(2, :)] + H(2, 4), ...
                    'ZData', [v1(3, :); v2(3, :)] + H(3, 4));
            end

        end

        function prepareJets(obj)
            % create 4 cylinders for the jets
            for i = 1:length(obj.jetFrameList)
                % 0.02 cylinder diameter
                [obj.X{i}, obj.Y{i}, obj.Z{i}] = cylinder(0.02, 6);
                obj.jets{i} = surf(obj.X{i}, obj.Y{i}, obj.Z{i}, 'FaceColor', 'r', 'FaceAlpha', 1);
            end

        end
        
        
        %update the aerodynamics forces for each link
        function updateAerodynamics_forces(obj,aerodynamics_forces_wb)
            
            for ii=1:length(obj.linkFrame)
                
                H_update = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, obj.linkFrame{ii}); %get homougeneous transformation matrix of each link
                
%               
%               X0_update{ii}=0;
%               Y0_update{ii}=0;
%               Z0_update{ii}=0;

%initial point of the aerodynamics force vector which is also the assumed
%application point of aerodynamcis forces
              X0_update=H_update(1,4); % origin of the link frame
              Y0_update=H_update(2,4);
              Z0_update=H_update(3,4);

              XA_update=aerodynamics_forces_wb(1,ii);
              YA_update=aerodynamics_forces_wb(2,ii);
              ZA_update=aerodynamics_forces_wb(3,ii);

                
               
            set(obj.aerodynamics_force_vector{ii},'XData',X0_update,'YData',Y0_update,'ZData',Z0_update,'UData',XA_update,...
                'VData',YA_update,'WData',ZA_update);
            end
        end
        
        function prepareAerodynamics_forces(obj) % pepare aerodynamics forces plotting
            
            ini_aero_forces_wb=zeros(3,13); % this value is not real initial aerodynamics force of robot but just to prepare the plot of aerodynamics
            %force vector
            
            
            for i=1:length(obj.linkFrame)
               
              H = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, obj.linkFrame{i});
              obj.X0{i}=H(1,4);
              obj.Y0{i}=H(2,4);
              obj.Z0{i}=H(3,4);
              
%               obj.X0{i}=0;
%               obj.Y0{i}=0;
%               obj.Z0{i}=0;
              obj.XA{i}=ini_aero_forces_wb(1,i);
              obj.YA{i}=ini_aero_forces_wb(2,i);
              obj.ZA{i}=ini_aero_forces_wb(3,i);
              
              obj.aerodynamics_force_vector{i}=quiver3(obj.X0{i},obj.Y0{i},obj.Z0{i},obj.XA{i},obj.YA{i},obj.ZA{i},'AutoScaleFactor',10);
              obj.aerodynamics_force_vector{i}.LineWidth=2;
              obj.aerodynamics_force_vector{i}.ShowArrowHead='on';
              
              
            end

              
             
        end
        
        
        
    end

end
