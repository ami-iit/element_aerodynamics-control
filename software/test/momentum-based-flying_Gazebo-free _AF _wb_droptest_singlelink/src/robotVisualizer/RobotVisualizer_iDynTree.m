classdef RobotVisualizer_iDynTree < matlab.System & matlab.system.mixin.CustomIcon
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
        KinDynModel, viz;
        g   = [0, 0, -9.81]; % gravity vector
        jetFrameList = {'chest_l_jet_turbine', 'chest_r_jet_turbine', 'l_arm_jet_turbine', 'r_arm_jet_turbine'};
        %modification for plotting aerodynamics forces
        linkFrame= {'head','chest','root_link','r_upper_arm','l_upper_arm',...
            'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'};
        scaling_factor = 0.5;
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()  % get obj.visualizer
                obj.prepareJets();
                %prepare the initial position of aerodynamics force vector
                obj.prepareAerodynamics_forces();
            end
        end
        
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Robot", "Visualizer"];
        end
        
        function stepImpl(obj, world_H_base, base_velocity, joints_positions, joints_velocity, jetIntensities,aerodynamics_forces_wb)
            
            if obj.config.visualizeRobot
                iDynTreeWrappers.setRobotState(obj.KinDynModel, world_H_base, joints_positions, base_velocity, joints_velocity, obj.g);
                if obj.viz.run()
                    obj.updateVisualization(world_H_base, joints_positions);
                    obj.updateJets(jetIntensities);
                    obj.updateAerodynamics_forces(aerodynamics_forces_wb);
                    obj.viz.draw()
                else
                    error('Closing visualizer.')
                end
            end
        end
        
        
        function updateVisualization(obj, world_H_base, joints_positions)
                s = iDynTree.VectorDynSize(obj.KinDynModel.NDOF);     
                for k = 0:length(joints_positions)-1
                    s.setVal(k,joints_positions(k+1));
                end
                baseRotation_iDyntree = iDynTree.Rotation();
                baseOrigin_iDyntree   = iDynTree.Position();
                T = iDynTree.Transform();
                for k = 0:2
                    baseOrigin_iDyntree.setVal(k,world_H_base(k+1,4));
                    for j = 0:2
                        baseRotation_iDyntree.setVal(k,j,world_H_base(k+1,j+1));                   
                    end
                end      
                T.setRotation(baseRotation_iDyntree);
                T.setPosition(baseOrigin_iDyntree);
                obj.viz.modelViz('iRonCub').setPositions(T, s)
        end
        
        
        function prepareRobot(obj)
            % Main variable of iDyntreeWrappers used for many things including updating
            % robot position and getting world to frame transforms
            % create KinDyn model
            obj.KinDynModel = iDynTreeWrappers.loadReducedModel(obj.config.jointOrder, 'root_link', ...
                obj.config.modelPath, obj.config.fileName, false);
            % instantiate iDynTree visualizer
            obj.viz = iDynTree.Visualizer();
            obj.viz.init();
            % if you're courious compile idyntree in devel and uncommend the line below
            % obj.viz.setColorPalette('meshcat');
            % add 'iRonCub' robot the the visualizer
            obj.viz.addModel(obj.KinDynModel.kinDynComp.model(), 'iRonCub')
            env = obj.viz.enviroment();
            env.setElementVisibility('floor_grid', true);
            env.setElementVisibility('world_frame', true);
            obj.viz.camera().animator().enableMouseControl(true);
            % adding lights
            obj.viz.enviroment().addLight('sun1');
            obj.viz.enviroment().lightViz('sun1').setType(iDynTree.DIRECTIONAL_LIGHT);
            obj.viz.enviroment().lightViz('sun1').setDirection(iDynTree.Direction(-1, 0, 0));
            obj.viz.enviroment().addLight('sun2')
            obj.viz.enviroment().lightViz('sun2').setType(iDynTree.DIRECTIONAL_LIGHT);
            obj.viz.enviroment().lightViz('sun2').setDirection(iDynTree.Direction(1, 0, 0));
        end
        
        function prepareJets(obj)
            % setting jets
            orange = iDynTree.ColorViz(1.0, 0.6, 0.1, 0.0);
            obj.viz.modelViz('iRonCub').jets().setJetColor(0, orange);
            obj.viz.modelViz('iRonCub').jets().setJetColor(1, orange);
            obj.viz.modelViz('iRonCub').jets().setJetColor(2, orange);
            obj.viz.modelViz('iRonCub').jets().setJetColor(3, orange);
            obj.viz.modelViz('iRonCub').jets().setJetsFrames(obj.jetFrameList);
            obj.viz.modelViz('iRonCub').jets().setJetsDimensions(0.02, 0.1, 0.3);
            obj.viz.modelViz('iRonCub').jets().setJetDirection(0, iDynTree.Direction(0, 0, 1.0));
            obj.viz.modelViz('iRonCub').jets().setJetDirection(1, iDynTree.Direction(0, 0, 1.0));
            obj.viz.modelViz('iRonCub').jets().setJetDirection(2, iDynTree.Direction(0, 0, 1.0));
            obj.viz.modelViz('iRonCub').jets().setJetDirection(3, iDynTree.Direction(0, 0, 1.0));
            
        end
        
        function prepareAerodynamics_forces(obj) 
            % prepare aerodynamics forces plotting            
            force = iDynTree.Direction();
            for i=1:length(obj.linkFrame)
                disp(obj.linkFrame{i})
                linkTransform = obj.KinDynModel.kinDynComp.getWorldTransform(obj.linkFrame{i});
                for j=1:3
                    % note that the indexing starts from 0 (not from 1) as
                    % in C++
                    force.setVal(j-1, 0);
                end
                obj.viz.vectors().addVector(linkTransform.getPosition(), force);
            end
        end 
        
        function updateJets(obj, jetIntensities)
            jet_int_iDyn = iDynTree.VectorDynSize(4);   
            max_jets_int = 220;
            for i=1:4
                jet_int_iDyn.setVal(i-1, jetIntensities(i)/max_jets_int);
            end
            obj.viz.modelViz('iRonCub').jets().setJetsIntensity(jet_int_iDyn);
        end
        
        
        function updateAerodynamics_forces(obj,aerodynamics_forces_wb)
            %update the aerodynamics forces for each link
            force = iDynTree.Direction();
            for i=1:length(obj.linkFrame)
                linkTransform = obj.KinDynModel.kinDynComp.getWorldTransform(obj.linkFrame{i});
                for j=1:3
                    % the single aerodynamics force is scaled by a constant factor
                    force.setVal(j-1, aerodynamics_forces_wb(j, i) * obj.scaling_factor);
                end
                obj.viz.vectors().updateVector(i-1, linkTransform.getPosition(), force);
            end
        end
    end
end
