classdef RobotVisualizer_iDynTree < matlab.System & matlab.system.mixin.CustomIcon
    % matlab.System handling the robot visualization
    % go in app/robots/iRonCub*/initVisualizer.m to change the setup config
    
    %@author: Giuseppe L'Erario  , Hui Tong
    
    properties (Nontunable)
        config;
        
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
        cfd_axis={'k','i','j'};
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            
            if obj.config.visualizeRobot
                % Perform one-time calculations, such as computing constants
                obj.prepareRobot()  % get obj.visualizer
                obj.prepareJets();
                %prepare the initial position of aerodynamics force vector
                obj.prepareAerodynamics_forces();
                
                %prepare the initial position of cfd frame origin
                obj.prepareCFD_body_frame();
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
                    obj.updateCFD_body_frame();
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
            
                %linkTransform = obj.KinDynModel.kinDynComp.getWorldTransform('root_link'); %choose root link frame origin as the virtual point to observe aerodynamic force
                o_com=iDynTreeWrappers.getCenterOfMassPosition(obj.KinDynModel);
                p_com=iDynTree.Position(o_com(1),o_com(2),o_com(3));
               
                for j=1:3
                    % note that the indexing starts from 0 (not from 1)
                    % as in C++
                    
                    force.setVal(j-1, 0);
                end
                %obj.viz.vectors().addVector(linkTransform.getPosition(), force);
                obj.viz.vectors().addVector(p_com, force);
        end 
        
        function prepareCFD_body_frame(obj)
            %prepare the created CFD frame basic elements (axis)
            axis = iDynTree.Direction();
            o_com=iDynTreeWrappers.getCenterOfMassPosition(obj.KinDynModel);
            p_com=iDynTree.Position(o_com(1),o_com(2),o_com(3));
            %frame_origin = obj.KinDynModel.kinDynComp.getWorldTransform('root_link');
            for i=1:length(obj.cfd_axis)
                disp(obj.cfd_axis{i})
                
                for j=1:3
                    % note that the indexing starts from 0 (not from 1)
                    % as in C++
                    
                    axis.setVal(j-1, 0);
                end
                obj.viz.vectors().addVector(p_com, axis);
            
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
            
            
                o_com=iDynTreeWrappers.getCenterOfMassPosition(obj.KinDynModel);
                p_com=iDynTree.Position(o_com(1),o_com(2),o_com(3));
                for j=1:3
                    % the single aerodynamics force is scaled by a constant factor
                    force.setVal(j-1, aerodynamics_forces_wb(j) * obj.scaling_factor);
                end
                obj.viz.vectors().updateVector(0, p_com, force);
            
                
        end
        
        function updateCFD_body_frame(obj)
            %update the created CFD frame basic elements (axis)
            axis = iDynTree.Direction();
            
            
            linkTransform_root = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'root_link');
            linkTransform_head = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'head');
            o_root=linkTransform_root(1:3,4);
            o_head=linkTransform_head(1:3,4);
            kaxis=(o_root-o_head)/norm(o_root-o_head);
            
            linkTransform_l = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'chest_l_jet_turbine');
            linkTransform_r = iDynTreeWrappers.getWorldTransform(obj.KinDynModel, 'chest_r_jet_turbine');
            o_l=linkTransform_l(1:3,4);
            o_r=linkTransform_r(1:3,4);
             o_r_to_l=o_l-o_r; % vector starting from o_r and point towards o_l
             proj=o_r_to_l-dot(o_r_to_l,kaxis)/(norm(kaxis)^2)*(kaxis);% projection of vector o_r_to_l 
             %on the plane that is normal to vector w_kaxis_cfd
             
             
             jaxis=proj/norm(proj); % j axis is defined
             
           
            
            iaxis=cross(jaxis,kaxis);
            cfd_frame=[kaxis iaxis jaxis];
            
            o_com=iDynTreeWrappers.getCenterOfMassPosition(obj.KinDynModel);
            p_com=iDynTree.Position(o_com(1),o_com(2),o_com(3));
            %frame_origin = obj.KinDynModel.kinDynComp.getWorldTransform('root_link');
            for i=1:length(obj.cfd_axis)
                for j=1:3
                    % the single axis vector is scaled by a constant factor
                    axis.setVal(j-1, cfd_frame(j, i)*0.5);
                end
                obj.viz.vectors().updateVector(i, p_com, axis);
            end
           
        end   
    end
end
