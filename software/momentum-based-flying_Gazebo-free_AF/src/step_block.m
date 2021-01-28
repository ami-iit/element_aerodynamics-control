ssclassdef step_block < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot

    % Public, tunable properties
    properties (Nontunable)
        robot_config;
        contact_config;
        jets_config;
        tStep; % time interaction of every step
        my_struct;
%         v_wind;rho;
    end

    properties (DiscreteState)

    end

    
    properties (Access = private)
        robot; contacts; state;
        af; % class for caculating aerodynamics forces
        jets;
        jets_frame containers.Map;
        generalized_external_wrenches;
   
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.robot = Robot(obj.robot_config);
            obj.contacts = Contacts(obj.contact_config.foot_print, obj.robot, obj.contact_config.friction_coefficient);
            obj.state = State(obj.tStep);
            obj.af=af(obj.v_wind,obj.rho);
            % using a conteiner map to access with a index to the relative jet frame
            obj.jets_frame = containers.Map([1, 2, 3, 4], {'l_arm_jet_turbine', 'r_arm_jet_turbine', 'chest_l_jet_turbine', 'chest_r_jet_turbine'});
            % instantiate 4 different jets - diffent coefficients
            for i = 1:4
                obj.jets{i} = Jet(obj.jets_config.coefficients(i, :), obj.jets_config.init_thrust(i), obj.tStep);
            end

            obj.state.set(obj.robot_config.initialConditions.w_H_b, obj.robot_config.initialConditions.s, ...
                obj.robot_config.initialConditions.base_pose_dot, obj.robot_config.initialConditions.s_dot);
        end

        function [w_H_b, s, base_pose_dot, s_dot, jet_intensities, wrench_left_foot, wrench_right_foot] = stepImpl(obj, jets_input, torque)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            
            % reset external wrenches
            obj.reset_external_wrenches();
            % add the external wrenches acting on the robot (more than jets
            % forces and contact forces) aerodynamics forces
            generalized_aerodynamics_wrench=obj.af.compute_gener_af(obj.robot,obj.state.base_pose_dot,obj.state.s_dot,'chest'); % chest frame
            % computing the jets forces
            [jet_intensities, generalized_jet_wrench] = obj.compute_jet_intensities_and_generalized_jet_wrench(jets_input);%jets_input could be jet throttle or intensity dot
            % computes the contact quantites and the velocity after a possible impact
            generalized_total_wrench = generalized_jet_wrench + generalized_aerodynamics_wrench;
            % update the total wrench with the computed contact forces and
            % update the state of the robot under contact with the ground
            
            
            [generalized_total_wrench, wrench_left_foot, wrench_right_foot, base_pose_dot, s_dot] = ...
                obj.contacts.compute_contact(obj.robot, torque, generalized_total_wrench, obj.state.base_pose_dot, obj.state.s_dot);
            % sets the velocity in the state
            obj.state.set_velocity(base_pose_dot, s_dot);
            % compute the robot acceleration
            [base_pose_ddot, s_ddot] = obj.robot.forward_dynamics(torque, generalized_total_wrench);
            % integrate the dynamics
            [w_H_b, s, base_pose_dot, s_dot] = obj.state.euler_step(base_pose_ddot, s_ddot);
            % update the robot state
            obj.robot.set_robot_state(w_H_b, s, base_pose_dot, s_dot) % inputs are accessed from State propertites
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
           obj.generalized_external_wrenches = zeros(obj.robot.NDOF + 6,1); 
        end

        function resetImpl(obj)

        end

        function [out, out2, out3, out4, out5, out6, out7] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [4 4]; % homogeneous matrix dim
            out2 = [23 1]; % joints position vector dim
            out3 = [6 1]; % base velocity vector dim
            out4 = [23 1]; % joints velocity vector dim
            out5 = [4 1]; % jet intensities vector dim
            out6 = [6 1]; % wrench left foot vector dim
            out7 = [6 1]; % wrench right foot vector dim
        end

        function [out, out2, out3, out4, out5, out6, out7] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
            out3 = "double";
            out4 = "double";
            out5 = "double";
            out6 = "double";
            out7 = "double";
        end

        function [out, out2, out3, out4, out5, out6, out7] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
        end

        function [out, out2, out3, out4, out5, out6, out7] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
        end

    end

end
