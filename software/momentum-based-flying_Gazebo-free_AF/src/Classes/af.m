classdef af < handle
    % the class af (aerodynamics forces) handles the computation of
    % generalized aerodynamics forces on one single link (' chest' in the first case).
    %rho:air density
    properties
        frame;% str
    end
    
    properties (Access = private)
        
        
        v_wind; % R^3
        
        rho;%R
        gama;
        Ka; %R
        
     %need contianer map in future to place all the robot links ,24 links ,
     %and set them into numbers
    end
    
    methods
        function obj=af(v_wind,rho)
            %af Construct an instance of this class
            %   robot - instance of Robot class 
            %   v_wind - the wind velocity expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.v_wind=v_wind;
            obj.rho=rho;
        end
        function set_Ka(obj)
            obj.gama=4; %set a random constant number for gama of chest as for now ,but it should be decided by frame
            obj.Ka=obj.rho*obj.gama*0.5;
            
        end
        
            
        
        
        function generalized_aerodynamics_wrench=compute_gener_af(obj,robot,base_pose_dot,s_dot,frame)
            % calculate generalized aerodynamics forces acting on one
            % single link
            % model
            generalized_aerodynamics_wrench = zeros(29, 1); % 29=23+6, n+ dof of floating base
            relative_velocity=obj.compute_relative_v(robot,base_pose_dot,s_dot,frame);
            w_kaxis_link=obj.compute_kaxis(robot,frame);
            AoA=obj.compute_AoA(relative_velocity,w_kaxis_link);
            C_D_link=obj.get_C_D(AoA); % also depends on frame name obj.frame
            C_L_link=obj.get_C_L(AoA);
            J=robot.get_frame_jacobian(frame);
            aerodynamics_forces=obj.compute_af_link(relative_velocity,C_D_link,C_L_link,AoA,w_kaxis_link);
            aerodynamics_wrench=[aerodynamics_forces;zeros(3,1)]; % torque is neglected 
            generalized_aerodynamics_wrench=generalized_aerodynamics_wrench+J'*aerodynamics_wrench;
            
            % only one single link frame is considered now 
        end
        function aerodynamics_forces=compute_af_link(obj,relative_velocity,C_D_link,C_L_link,AoA,w_kaxis_link)
            
            % calculate aerodynamics forces in world frame since all the
            % vectors are expressed in world frame
      
            aerodynamics_forces=-obj.Ka*norm(relative_velocity)*((C_D_link+C_L_link*cot(AoA))*relative_velocity+C_L_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link);
            % aerodynamics_forces \in R^3
            
        
        end
        
        function relative_velocity=compute_relative_v(robot,base_pose_dot,s_dot,frame) %base__pose_dot and s_dot are from State class 
            %this function computes the relative velocity between linear
            %velocity of link frame origin w.r.t inertial frame and the
            %wind velocity expressed in the inertial frame
            robot_velocity=[base_pose_dot;s_dot];
            J=robot.get_frame_jacobian(frame);
            linear_velocity_link=J(1:3,:)*robot_velocity;
            relative_velocity=linear_velocity_link-obj.v_wind; %expressed in inertial coordinate 
        end
        function w_kaxis_link=compute_kaxis(robot,frame)  % unit vector of link frame expressed in inertial orientation
            w_H_link=robot.get_frame_H(frame);
            w_kaxis_link=w_H_link(1:3,3);  % double check the defination of orientation matrix
        end
        function AoA=compute_AoA(w_kaxis_link,relative_velocity)
            
            AoA_inDegree=atan2(norm(cross(relative_velocity,w_kaxis_link)),dot(relative_velocity,w_kaxis_link));
            AoA=deg2rad(AoA_inDegree);
        end
            
        function C_D_link=get_C_D(obj,AoA) %unset
            %C_D_link is related to the value of AoA and frame property
            C_D_link=obj.frame*0+AoA*0+1; % since the function of C_D related to frame and AoA is still unknown, a random linear equation is used 
        end
        function C_L_link=get_C_L(obj,AoA) %unset
            C_L_link=obj.frame*0+AoA*0+1;
        end
        
        
        
        
        
    end
end
