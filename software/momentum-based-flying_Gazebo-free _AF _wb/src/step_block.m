classdef step_block < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    % Public, tunable properties
    properties (Nontunable)
        robot_config;
        contact_config;
        jets_config;
        tStep; 
        aerodynamics_config;
    end

    properties (DiscreteState)

    end

    
    properties (Access = private)
        robot; contacts; state;
        aerodynamics; % object for caculating aerodynamics forces
        jets;
        jets_frame containers.Map;
        
        link_frame containers.Map; % the frame we consider to add aerodynamics forces on, up to now only 'chest' 
        axis_frame containers.Map;
        
        generalized_external_wrenches;% external wrenches (more than jets forces and contact forces), aerodynamics forces in this case
        
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = Robot(obj.robot_config);
            obj.contacts = Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient);
            obj.state = State(obj.tStep);
            
            % using a conteiner map to access with a index to the relative jet frame
            obj.jets_frame = containers.Map([1, 2, 3, 4], {'l_arm_jet_turbine', 'r_arm_jet_turbine', 'chest_l_jet_turbine', 'chest_r_jet_turbine'});
            obj.link_frame = containers.Map([1,2,3,4,5,6,7,8,9,10,11,12,13],{'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'}) ;% set frame on 
            obj.axis_frame = containers.Map([1,2,3,4,5,6,7,8,9,10,11,12,13],{'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_arm_jet_turbine','l_arm_jet_turbine','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'}) ;%frames for getting symmetric axis
            
            
            obj.aerodynamics=Aerodynamics_force_link(obj.aerodynamics_config);% object of Af class with Af_config as input
            % instantiate 4 different jets - diffent coefficients
            for i = 1:4
                obj.jets{i} = Jet(obj.jets_config.coefficients(i, :), obj.jets_config.init_thrust(i), obj.tStep);
            end

            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, jet_intensities, wrench_left_foot, wrench_right_foot,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb] = stepImpl(obj, jets_input, torque)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % reset external wrenches
            obj.reset_external_wrenches();
            % add the external wrenches acting on the robot (more than jets
            % forces and contact forces) aerodynamics forces
            
            [generalized_aerodynamics_wb,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb]=obj.compute_aero_wholebody();
            
            
            %compute generalized aerodynamics wrench on whole body 
            obj.add_external_wrench(generalized_aerodynamics_wb);% compute extra external wrenches
            %obj.add_external_wrench(zeros(6,1));
            % computing the jets forces
            [jet_intensities, generalized_jet_wrench] = obj.compute_jet_intensities_and_generalized_jet_wrench(jets_input);%jets_input could be jet throttle or intensity dot
            % computes the contact quantites and the velocity after a possible impact
            generalized_total_wrench = generalized_jet_wrench + obj.generalized_external_wrenches;
            % update the total wrench with the computed contact forces and
            % update the state of the robot under contact with the ground
            
            
%             [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
%                 obj.contacts.compute_contact(obj.robot, torque, generalized_total_wrench, obj.state.base_pose_dot, obj.state.s_dot);
            wrench_left_foot = zeros(6,1);
            wrench_right_foot = zeros(6,1);
            base_pose_dot = obj.state.base_pose_dot;
            s_dot = obj.state.s_dot;
            % sets the velocity in the state
            obj.state.set_velocity(base_pose_dot, s_dot);
            % compute the robot acceleration
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench);
            % integrate the dynamics
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.euler_step(base_pose_ddot, s_ddot);
            % update the robot state
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot) % inputs are accessed from State propertites
        end
        
        function [generalized_aerodynamics_wb,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb]=compute_aero_wholebody(obj)
            
            % for whole body aerodynamics forces, 10 links are considered
            generalized_aerodynamics_wb=zeros(29,1);
            aerodynamics_forces_wb=zeros(3,obj.aerodynamics.N_link);
            relative_velocity_wb=zeros(3,obj.aerodynamics.N_link);
            
            AoA_wb=zeros(1,obj.aerodynamics.N_link);
            
            
            for i=1:obj.aerodynamics.N_link
                relative_velocity_wb(1:3,i)=obj.aerodynamics.compute_relative_v(obj.robot,obj.state.base_pose_dot,obj.state.s_dot,obj.link_frame(i));
                
                ini_relative_velocity=obj.aerodynamics.compute_relative_v(obj.robot,zeros(6,1),zeros(23,1),obj.link_frame(i));
                w_kaxis_link=obj.aerodynamics.compute_kaxis(obj.robot,obj.axis_frame(i));
                AoA_wb(i)=obj.aerodynamics.compute_AoA(w_kaxis_link,relative_velocity_wb(1:3,i));
                
                 aerodynamics_forces_single=obj.aerodynamics.compute_af_link(ini_relative_velocity,w_kaxis_link,obj.link_frame(i));
%                aerodynamics_forces_single=obj.aerodynamics.compute_af_link(relative_velocity_wb(1:3,i),w_kaxis_link,obj.link_frame(i));
                aerodynamics_forces_wb(1:3,i)=aerodynamics_forces_single;
                %  whole body aerodynamics forces distributed on different
                %  links
                
                aerodynamics_wrench_single=obj.aerodynamics.compute_gener_af(obj.robot,aerodynamics_forces_single,obj.link_frame(i));
                %generalized aerodynamics wrench for one single link
                
                generalized_aerodynamics_wb=generalized_aerodynamics_wb+aerodynamics_wrench_single;% total aerodynamics wrench
                
               
            
            end
          
        end
        
        
        function [jet_intensities, generalized_jet_wrench] = compute_jet_intensities_and_generalized_jet_wrench(obj, u)
            % u is jet input
            generalized_jet_wrench = zeros(29, 1); % 29=23+6, n+ dof of floating base
            jet_intensities = zeros(4, 1);
            % compute for every jet the thrust and the generalized wrench contribution
            % using the relative frame (the container map is used here)
            for i = 1:4
                if obj.jets_config.use_jet_dyn % decide to use jet dynamics or not , if no then we can get T from intergration of dotT directly
                    jet_intensities(i) = obj.jets{i}.get_thrust(u(i));
                else
                    jet_intensities(i) = obj.jets{i}.get_thrust_from_dot_T(u(i));
                end
                f = obj.compute_jet_force_in_world_frame(jet_intensities(i), obj.jets_frame(i));
                generalized_jet_wrench = generalized_jet_wrench + obj.compute_generalized_wrench([f; zeros(3,1)], obj.jets_frame(i));
            end

        end

        function f = compute_jet_force_in_world_frame(obj, t, frame)
            %compute_jet_force_in_world_frame returns the jet force in the
            % world frame
            H = obj.robot.get_frame_H(frame); %homogenous transformation matrix
            % represent the (pure) z force in the world
            f = -H(1:3, 3) * t; 
        end

        function f = compute_generalized_wrench(obj, wrench, frame) %transfer wrench from body frame to the base frame
            %compute_generalized_wrench compute the generalized wrench contribution
            J = obj.robot.get_frame_jacobian(frame);% get jacobian for specific frame , which has the same function as the block in simulink does
            f = J' * wrench;
        end
        
        
        
        function reset_external_wrenches(obj)
           obj.generalized_external_wrenches = zeros(obj.robot.NDOF + 6,1); %set size 
        end
        function add_external_wrench(obj, otherwrench)
            obj.generalized_external_wrenches  = obj.generalized_external_wrenches + otherwrench;
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [23 1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [23 1]; % joints velocity vector dim
            out5 = [4 1]; % jet intensities vector dim
            out6 = [6 1]; % wrench left foot vector dim
            out7 = [6 1]; % wrench right foot vector dim
            out8 = [3 13];% aerodynamics forces vector dim
            out9 = [3 13];% relative velocity vector dim
            out10 = [1 13];% AoA dim
           
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = "double";
            out7 = "double";
            out8 = "double";
            out9 = "double";
            out10 = "double";
           
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;
            out9 = false;
            out10 = false;
           
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;
            out9 = true;
            out10 = true;
            
        end

    end


end
