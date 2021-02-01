classdef Af < handle
    % the class Af (aerodynamics forces) handles the computation of
    %  aerodynamics forces 
    %rho:air density
    %gama : shape coefficient
    %Ka : Ka = rho*gama/2
    %C_D_link: drag coefficient
    %C_L_link:lift coefficient
    
    
    properties (Access = private)
        
        
        v_wind; % R^3
        
        rho;%R
        gama;
        Ka; %R
        C_D_link;
        C_L_link;
        
     %need contianer map in future to place all the robot links ,
     
    end
    
    methods  
        function obj=Af(af_config)
            %af Construct an instance of this class
            %   v_wind - the wind velocity vector expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.v_wind=af_config.v_wind;
            obj.rho=af_config.rho;
            obj.gama=af_config.gama;
            obj.Ka=af_config.Ka;
            obj.C_D_link=af_config.C_D_link;
            obj.C_L_link=af_config.C_L_link;
        end
        
        
            
        %% F_a=-Ka*|v_a|*((C_D()+C_L()cot(AoA))*v_a+C_L()/sin(AoA)*|v_a|*k) formula of computing aerodynamics forces on each link
        
        function generalized_aerodynamics_wrench=compute_gener_af(obj,robot,base_pose_dot,s_dot,frame)
            % calculate generalized aerodynamics forces acting on one
            % single link
            %generalized_aerodynamics_wrench = zeros(29, 1); % 29=23+6, n+ dof of floating base
            relative_velocity=obj.compute_relative_v(robot,base_pose_dot,s_dot,frame);% v_a
            w_kaxis_link=obj.compute_kaxis(robot,frame);% k unit vector of link frame [0 0 1] expressed in world orientation frame
            AoA=obj.compute_AoA(relative_velocity,w_kaxis_link);%angle of attck
            
            J=robot.get_frame_jacobian(frame); % jacobian of specific frame 
            aerodynamics_forces=obj.compute_af_link(relative_velocity,obj.C_D_link,obj.C_L_link,AoA,w_kaxis_link);
            aerodynamics_wrench=[aerodynamics_forces;zeros(3,1)]; % aerodynamics torque effects are neglected 
            generalized_aerodynamics_wrench=J'*aerodynamics_wrench;
            
            % only one single link frame is considered now 
        end
    end
    
    methods (Access = private)
        
        
        function aerodynamics_forces=compute_af_link(obj,relative_velocity,AoA,w_kaxis_link)
            
            % calculate aerodynamics forces expressed as a vector in world frame since all the
            % vectors are expressed in world frame
      
            aerodynamics_forces=-obj.Ka*norm(relative_velocity)*((obj.C_D_link+obj.C_L_link*cot(AoA))*relative_velocity+obj.C_L_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link);
            % aerodynamics_forces \in R^3
            
        
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
        function w_kaxis_link=compute_kaxis(robot,frame)  % unit vector of link frame expressed in inertial orientation
            w_H_link=robot.get_frame_H(frame); % 4X4
            w_kaxis_link=zeros(3,1);
            w_kaxis_link(1)=w_H_link(1,3);
           % double check the defination of orientation matrix
           w_kaxis_link(2)=w_H_link(2,3);
           w_kaxis_link(3)=w_H_link(3,3);
        end
        function AoA=compute_AoA(w_kaxis_link,relative_velocity) % the angle between relative velocity and k axis is defined as angle of attack
            
            AoA=atan2(norm(cross(relative_velocity,w_kaxis_link)),dot(relative_velocity,w_kaxis_link));
            
        end
            
        
        
        
        
        
    end
end
