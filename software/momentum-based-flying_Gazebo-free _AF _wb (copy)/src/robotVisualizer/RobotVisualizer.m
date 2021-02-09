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
        linkFrame= {'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'};
        
        aerodynamics_force_vector  ;
       
        X0,Y0,Z0;
        XA,YA,ZA;
    end

    methods (Access = protected)

        function setupImpl(obj)
            

            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()  % get obj.visualizer

                if obj.config.visualizeJets
                    obj.prepareJets();
                end
             obj.prepareAerodynamics_forces();
            end
           
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end

        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity, jetIntensities,aerodynamics_forces_wb,v_wind)

            %          plot wind velocity vector with beginning point at world frame origin
%             obj.prepareWind_Velocity(v_wind);
%             obj.prepareWind_Velocity(v_wind);
           
            if obj.config.visualizeRobot
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                iDynTreeWrappers.updateVisualization(obj.KinDynModel, obj.visualizer);
                obj.followTheRobot();

                if obj.config.visualizeJets
                    obj.updateJets(jetIntensities);
                end
           
            obj.updateAerodynamics_forces(aerodynamics_forces_wb);
            
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
                'color', [1, 1, 1], 'material', 'metal', 'transparency', 1, 'debug', true, 'view', obj.pov, ...
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
        
        
        function updateAerodynamics_forces(obj,aerodynamics_forces_wb)
            
            for ii=1:length(obj.linkFrame)
                
                H_update = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, obj.linkFrame{ii});
                
              X0_update{ii}=H_update(1,4);
              Y0_update{ii}=H_update(2,4);
              Z0_update{ii}=H_update(3,4);
              XA_update{ii}=aerodynamics_forces_wb(1,ii);
              YA_update{ii}=aerodynamics_forces_wb(2,ii);
              ZA_update{ii}=aerodynamics_forces_wb(3,ii);
                
                %'XData',X0_update{ii},'YData',Y0_update{ii},'ZData',Z0_update{ii},
            set(obj.aerodynamics_force_vector{ii},'XData',X0_update{ii},'YData',Y0_update{ii},'ZData',Z0_update{ii},'UData',XA_update{ii},...
                'VData',YA_update{ii},'WData',ZA_update{ii});
            end
        end
        
        function prepareAerodynamics_forces(obj)
            
            ini_aero_forces_wb=zeros(3,13);
            
            
            
            for i=1:length(obj.linkFrame)
               
              H = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, obj.linkFrame{i});
              obj.X0{i}=H(1,4);
              obj.Y0{i}=H(2,4);
              obj.Z0{i}=H(3,4);
              obj.XA{i}=ini_aero_forces_wb(1,i);
              obj.YA{i}=ini_aero_forces_wb(2,i);
              obj.ZA{i}=ini_aero_forces_wb(3,i);
              
              obj.aerodynamics_force_vector{i}=quiver3(obj.X0{i},obj.Y0{i},obj.Z0{i},obj.XA{i},obj.YA{i},obj.ZA{i},'AutoScaleFactor',0.08);
              obj.aerodynamics_force_vector{i}.LineWidth=2;
              obj.aerodynamics_force_vector{i}.ShowArrowHead='on';
              
              
            end

              
             
        end
        
        function prepareWind_Velocity(obj,v_wind)
%             H = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'base_link');
%             X_0=H(1,4);
%             Y_0=H(2,4)+0.5;
%             Z_0=H(3,4);
            X_0=0;
            Y_0=0.5;
            Z_0=0.5;
            q1=quiver3(X_0:X_0+0.2,Y_0:Y_0+0.2,Z_0:Z_0+0.2,v_wind(1)+X_0:v_wind+X_0+0.2,v_wind(2)+Y_0:v_wind(2)+Y_0+0.2,v_wind(3)+Z_0:v_wind(3)+Z_0+0.2,'AutoScaleFactor',0.05,'Color','r','LineWidth',4,'ShowArrowHead','on');
%             q2=quiver3(X_0,Y_0,Z_0-0.05,v_wind(1)+X_0,v_wind(2)+Y_0,v_wind(3)+Z_0-0.05,'AutoScaleFactor',0.05,'Color','r','LineWidth',4,'ShowArrowHead','on');
%             q3=quiver3(X_0,Y_0,Z_0-0.1,v_wind(1)+X_0,v_wind(2)+Y_0,v_wind(3)+Z_0-0.1,'AutoScaleFactor',0.05,'Color','r','LineWidth',4,'ShowArrowHead','on');
            
%             pause(0.00001);
%              delete(q1);
%              delete(q2);
%              delete(q3);
        end
        
    end

end
