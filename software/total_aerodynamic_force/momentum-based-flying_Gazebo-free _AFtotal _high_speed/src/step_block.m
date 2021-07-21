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
        
        link_frame containers.Map; % the frame we consider to add aerodynamics forces on
        
        
        generalized_external_wrenches;% external wrenches (more than jets forces and contact forces), aerodynamics forces in this case
        
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = Robot(obj.robot_config);
            obj.contacts = Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient);
            obj.state = State(obj.tStep);
            
            % using a conteiner map to access with a index to the relative jet frame
            obj.jets_frame = containers.Map([1, 2, 3, 4], {'l_arm_jet_turbine', 'r_arm_jet_turbine', 'chest_l_jet_turbine', 'chest_r_jet_turbine'});
            obj.link_frame = containers.Map([1,2,3,4,5,6,7,8,9,10,11,12,13,14],{'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot','com'}) ;% set frame on 
            
            
            
            obj.aerodynamics=Aerodynamics_force_link(obj.aerodynamics_config);% object of Aerodynamics_force_link class with aerodynamics_config as input
            % instantiate 4 different jets - diffent coefficients
            for i = 1:4
                obj.jets{i} = Jet(obj.jets_config.coefficients(i, :), obj.jets_config.init_thrust(i), obj.tStep);
            end

            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, jet_intensities, wrench_left_foot, wrench_right_foot,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb,beta,Fa_drag,Fa_normal] = stepImpl(obj, jets_input, torque, v_wind)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % reset external wrenches
            obj.reset_external_wrenches();
            % add the external wrenches acting on the robot (more than jets
            % forces and contact forces) aerodynamics forces
            
            [generalized_aerodynamics_wb,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb,beta,Fa_drag,Fa_normal]=obj.compute_aero_wholebody(v_wind);
            
            
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
        
        function [generalized_aerodynamics_wb,aerodynamics_forces_wb,relative_velocity_wb,AoA_wb,beta,Fa_drag,Fa_normal]=compute_aero_wholebody(obj,v_wind)
                
                %center of mass velocity is used to present the robot
                %linear velocity
                relative_velocity_wb=obj.aerodynamics.get_com_va(obj.robot,obj.state.base_pose_dot,obj.state.s_dot,v_wind);% Va_com
                [w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd]=obj.aerodynamics.cfd_body_frame(obj.robot);% defined body frame
                AoA_wb=obj.aerodynamics.compute_AoA(w_kaxis_cfd,relative_velocity_wb);%defined alpha angle
                beta=obj.aerodynamics.compute_beta(w_kaxis_cfd,w_iaxis_cfd,relative_velocity_wb); %defined beta angle
                
                [w_xaxis_va,w_yaxis_va,w_zaxis_va]=obj.aerodynamics.cfd_velocity_frame(w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd,beta,AoA_wb); % relative velocity frame
                [Fa_total,Fa_drag,Fa_side,Fa_normal]=obj.aerodynamics.compute_total_af(w_kaxis_cfd,w_iaxis_cfd,w_xaxis_va,w_yaxis_va,w_zaxis_va,relative_velocity_wb); %total aerodynamic force

                aerodynamics_forces_wb=Fa_total;
               
                J_com=obj.robot.get_com_jacobian(); % 3x29 CoM jacobian
                generalized_aerodynamics_wb=J_com'*aerodynamics_forces_wb;% 29x1 generalized aerodynamic wrench 
                
        
        end
        
        
        function [jet_intensities, generalized_jet_wrench] = compute_jet_intensities_and_generalized_jet_wrench(obj, u)
            % u is jet input
            generalized_jet_wrench = zeros(obj.robot.NDOF+6, 1); %  n+ dof of floating base
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

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10,out11,out12,out13] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [23 1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [23 1]; % joints velocity vector dim
            out5 = [4 1]; % jet intensities vector dim
            out6 = [6 1]; % wrench left foot vector dim
            out7 = [6 1]; % wrench right foot vector dim
            out8 = [3 1];% aerodynamics forces vector dim
            out9 = [3 1];% relative velocity vector dim
            out10 = [1 1];% AoA dim
            out11 = [1 1];% beta dim
            out12 = [3 1];% aerodynamic drag force vector dim
            out13 = [3 1];% aerodynamic normal force vector dim
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10,out11,out12,out13] = getOutputDataTypeImpl(~)
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
            out11 = "double";
            out12 = "double";
            out13 = "double";
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10,out11,out12,out13] = isOutputComplexImpl(~)
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
            out11 = false;
            out12 = false;
            out13 = false;
        end

        function [out, out2, out3, out4, out5, out6, out7,out8,out9,out10,out11,out12,out13] = isOutputFixedSizeImpl(~)
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
            out11 = true;
            out12 = true;
            out13 = true;
        end

    end


end