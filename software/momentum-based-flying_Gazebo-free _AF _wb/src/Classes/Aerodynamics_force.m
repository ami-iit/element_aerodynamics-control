classdef Aerodynamics_force < handle
    % the class Af (aerodynamics forces) handles the computation of
    %  aerodynamics forces 
    %rho:air density
    %gama : shape coefficient
    %Ka : Ka = rho*gama/2
    %C_D_link: drag coefficient
    %C_L_link:lift coefficient
    
    properties
        N_link;
    end
    
    properties (Access = private)
        
        v_wind; % R^3
        
        rho;%R
        gama (13,1) double;
        Ka (13,1) double; %R
        C_D;
        C_L;
        
     %need contianer map in future to place all the robot links ,
     
    end
    
    methods  
        function obj=Aerodynamics_force(aerodynamics_config)
            %af Construct an instance of this class
            %   v_wind - the wind velocity vector expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.N_link=aerodynamics_config.NOL;
            obj.v_wind=aerodynamics_config.v_wind;
            obj.rho=aerodynamics_config.rho;
            obj.gama=aerodynamics_config.gama;%vector 13X1
            obj.Ka=aerodynamics_config.Ka;%vector 13X1
            obj.C_D=aerodynamics_config.C_D;%vector 13X1
            obj.C_L=aerodynamics_config.C_L;%vector 13X1
        end
        
        
            
        % F_a=-Ka*|v_a|*((C_D()+C_L()cot(AoA))*v_a+C_L()/sin(AoA)*|v_a|*k) formula of computing aerodynamics forces on each link
        
        function generalized_aerodynamics_wrench=compute_gener_af(obj,robot,aerodynamics_forces,frame)
            % calculate generalized aerodynamics forces acting on one
            % single link
            %generalized_aerodynamics_wrench = zeros(29, 1); % 29=23+6, n+ dof of floating base
            
            
            J=robot.get_frame_jacobian(frame); % jacobian of specific frame 
            
            %aerodynamics_wrench=[aerodynamics_forces;zeros(3,1)]; % aerodynamics torque effects are neglected 
            generalized_aerodynamics_wrench=J'*[aerodynamics_forces;zeros(3,1)];
            
            % only one single link frame is considered now 
        end
    end
    
    methods 
        
        
        function aerodynamics_forces=compute_af_link(obj,relative_velocity,w_kaxis_link,frame)
            
            % calculate aerodynamics forces expressed as a vector in world frame since all the
            % vectors are expressed in world frame
            [Ka_link,C_D_link,C_L_link]=obj.get_coeff(frame);
            AoA=obj.compute_AoA(relative_velocity,w_kaxis_link);
            aerodynamics_forces=-Ka_link*norm(relative_velocity)*((C_D_link+C_L_link*cot(AoA))*relative_velocity+C_L_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link);
            % aerodynamics_forces \in R^3
            
        
        end
        
        function [Ka_link,C_D_link,C_L_link]=get_coeff(obj,frame) % get force coefficients and shape coefficient for one link
            frame_number = containers.Map({'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1','l_elbow_1','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'},[1,2,3,4,5,6,7,8,9,10,11,12,13]);
            Ka_link=obj.Ka(frame_number(frame));
            C_D_link=obj.C_D(frame_number(frame));
            C_L_link=obj.C_L(frame_number(frame));
        end
        
        function relative_velocity=compute_relative_v(obj,robot,base_pose_dot,s_dot,frame) %base__pose_dot and s_dot are from the state before forward dynamics
            %this function computes the relative velocity between linear
            %velocity of link frame origin w.r.t inertial frame and the
            %wind velocity expressed in the inertial frame
            
            robot_velocity=[base_pose_dot;s_dot]; %29X1
            J=robot.get_frame_jacobian(frame);% 6X29
            link_velocity=J*robot_velocity;% 6X1
            linear_velocity_link=link_velocity(1:3);
            relative_velocity=linear_velocity_link-obj.v_wind; %expressed in world coordinate 
        end
        function w_kaxis_link=compute_kaxis(obj,robot,frame)  % unit vector of link frame expressed in inertial orientation
            w_H_link=robot.get_frame_H(frame); % 4X4
            
            w_kaxis_link=w_H_link(1:3,3);
           
           
        end
        function AoA=compute_AoA(obj,w_kaxis_link,relative_velocity) % the angle between relative velocity and k axis is defined as angle of attack
            
            AoA=atan2(norm(cross(relative_velocity,w_kaxis_link)),dot(relative_velocity,w_kaxis_link));
            
        end
            
        
        
        
        
        
    end
end
