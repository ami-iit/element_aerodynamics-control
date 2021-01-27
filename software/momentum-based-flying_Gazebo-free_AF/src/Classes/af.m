classdef af < handle
    % the class af (aerodynamics forces) handles the computation of
    % generalized aerodynamics forces on one single link (' chest' in this case).
    %rho:air density
    
    properties 
        
        
        v_wind; 
        
        rho;
        gama;
        Ka_chest;
        AoA; % angle of attack
     %need contianer map in future to place all the robot links ,24 links ,
     %and set them into numbers
    end
    
    methods
        function obj=af(v_wind,rho,frame)
            %af Construct an instance of this class
            %   robot - instance of Robot class 
            %   v_wind - the wind velocity expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.v_wind=v_wind;
            obj.rho=rho;
            obj.frame=frame;
            obj.get_gama(frame);
            obj.Ka=rho*obj.gama*0.5;
            
        end
        function get_gama(obj)
            obj.gama=4; %set a random constant number for gama of chest ,should be decided by obj.frame
        end
        
        function generalized_aerodynamics_force=compute_af(obj,robot,base_pose_dot,s_dot,relative_velocity,C_D_link,C_L_link,AoA,w_kaxis_link,aerodynamics_forces)
            % calculate generalized aerodynamics forces acting on one
            % single link
            % model
            J=robot.get_frame_jacobian(obj.frame);
            aerodynamics_forces=obj.compute_af_link(base_pose_dot,s_dot,relative_velocity,C_D_link,C_L_link,AoA,w_kaxis_link);
            generalized_aerodynamics_force=J'*aerodynamics_forces;
            
            
        end
        function aerodynamics_forces=compute_af_link(obj,robot,base_pose_dot,s_dot);
            relative_velocity=obj.compute_relative_v(robot,base_pose_dot,s_dot);
            w_kaxis_link=obj.compute_kaxis(robot);
            AoA=obj.compute_AoA(robot,relative_velocity,w_kaxis_link);
            C_D_link=obj.get_C_D(obj.AoA); % also depends on frame name
            C_L_link=obj.get_C_L(obj.AoA);
            
            aerodynamics_forces=-obj.Ka*norm(relative_velocity)*((C_D_link+C_L_link*cot(AoA))*relative_velocity+C_L_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link));
            
            
        
        end
        
        function relative_velocity=compute_relative_v(obj,robot,base_pose_dot,s_dot) %base__pose_dot and s_dot are from State class 
            %this function computes the relative velocity between linear
            %velocity of link frame origin w.r.t inertial frame and the
            %wind velocity expressed in the inertial frame
            robot_velocity=[base_pose_dot;s_dot];
            J=robot.get_frame_jacobian(obj.frame);
            linear_velocity_link=J(1:3,:)*robot_velocity;
            relative_velocity=linear_velocity_link-obj.v_wind; %expressed in inertial coordinate 
        end
        function w_kaxis_link=compute_kaxis(obj,robot)  % unit vector of link frame expressed in inertial orientation
            w_H_link=robot.get_frame_H(obj.frame);
            w_kaxis_link=w_H_link(1:3,3);  % double check the defination of orientation matrix
        end
        function AoA=compute_AoA(obj,robot,relative_velocity,w_kaxis_link)
            w_kaxis_link=obj.compute_kaxis(robot);
            AoA_inDegree=atan2(norm(cross(relative_velocity,w_kaxis_link)),dot(relative_velocity,w_kaxis_link));
            AoA=deg2rad(AoA_inDegree);
        end
            
        function C_D_link=get_C_D(obj,AoA)
            %C_D_link is related to the value of AoA and frame property
        end
        function C_L_link=get_C_L(obj,AoA)
        end
        
        
        
        
        
    end
end
