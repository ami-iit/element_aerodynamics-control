classdef Aerodynamics_force_link < handle
    % the class Aaerodynamics_forces handles the computation of
    %  aerodynamics forces on one single link
    %rho:air density
    %gama : shape coefficient
    %Ka : Ka = rho*gama/2
    %C_D_link: drag coefficient
    %C_L_link:lift coefficient
    %link list:  ['head'=1,
    %,'chest'=2,'root_link'=3,'r_upper_arm'=4,'l_upper_arm'=5,'r_elbow_1_aero_frame'=6,
    %'l_elbow_1_aero_frame'=7,'r_upper_leg'=8,'l_upper_leg'=9,'r_lower_leg'=10,
    %'l_lower_leg'=11,'r_foot'=12,'l_foot'=13]
    
    properties
        N_link;
    end
    
    properties (Access = private)
        
        v_wind (3,1) double ; % R^3
        
        rho;%R
        gama (13,1) double;
        Ka (13,1) double; 
        C_D;
        C_L;
    end
     
    
    
    methods  
        function obj=Aerodynamics_force_link(aerodynamics_config)
            %af Construct an instance of this class
            %   v_wind - the wind velocity vector expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.N_link=aerodynamics_config.NOL;
            obj.v_wind=aerodynamics_config.v_wind;
            obj.rho=aerodynamics_config.rho;
            obj.gama=aerodynamics_config.gama;%vector N_linkX1
            obj.Ka=aerodynamics_config.Ka;%vector N_linkX1
            obj.C_D=aerodynamics_config.C_D;%vector 
            obj.C_L=aerodynamics_config.C_L;%vector 
        end
        
        
            
        % F_a=-Ka*|v_a|*((C_D()+C_L()cot(AoA))*v_a+C_L()/sin(AoA)*|v_a|*w_kaxis) formula of computing aerodynamics forces on each link
        
        function generalized_aerodynamics_wrench=compute_gener_af(obj,robot,aerodynamics_forces,frame)
            % calculate generalized aerodynamics forces acting on one
            % single link which transfers aerodynamics forces from COM of
            % the link to the base frame of robot
            %generalized_aerodynamics_wrench = zeros(6+NDOF, 1); %  n+ dof of floating base
            
           
            J=robot.get_frame_jacobian(frame); % jacobian of specific frame 
            
            %aerodynamics_wrench=[aerodynamics_forces;zeros(3,1)]; % aerodynamics torque effects are neglected 
            generalized_aerodynamics_wrench=J'*[aerodynamics_forces;zeros(3,1)];
            
            % only one single link frame is considered now 
        end
    end
    
    methods 
        
        
        function aerodynamics_forces=compute_af_link(obj,relative_velocity,w_kaxis_link,frame)
            
            % calculate aerodynamics forces (R^3) expressed as a vector in world frame since all the
            % vectors in formula are expressed in world frame
            
           
            [Ka_link,C_D_link,C_L_link]=obj.get_coeff(frame); % get coefficients of one single link
            AoA=obj.compute_AoA(relative_velocity,w_kaxis_link);% compute angle of attack for a specific link
            AoA = max(AoA, 1e-8); %set tolerance to avoid Inf value 
            
            aerodynamics_forces=-Ka_link*norm(relative_velocity)*((C_D_link+C_L_link*cot(AoA))*relative_velocity+C_L_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link);
            % aerodynamics_forces \in R^3
            
        
        end
        
        function [Ka_link,C_D_link,C_L_link]=get_coeff(obj,frame) % get force coefficients and shape coefficient for one link
            
            frame_number = containers.Map({'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'},[1,2,3,4,5,6,7,8,9,10,11,12,13]);
            Ka_link=obj.Ka(frame_number(frame));
            C_D_link=obj.C_D(frame_number(frame));
            C_L_link=obj.C_L(frame_number(frame));
        end
        
        function relative_velocity=compute_relative_v(obj,robot,base_pose_dot,s_dot,frame) %base__pose_dot and s_dot are from the state before forward dynamics
            %this function computes the relative velocity between linear
            %velocity of link frame origin w.r.t inertial frame and the
            %wind velocity expressed in the inertial frame
            
            
            robot_velocity=[base_pose_dot;s_dot]; %(Ndof+6)X1
            J=robot.get_frame_jacobian(frame);% 6X(Ndof+6)
            link_velocity=J*robot_velocity;% 6X1 link velocity (linear and angular) w.r.t inertial frame
            linear_velocity_link=link_velocity(1:3);
            
            
            
                relative_velocity=linear_velocity_link-obj.v_wind; %expressed in world coordinate 
          
        end
        function w_kaxis_link=compute_kaxis(obj,robot,frame)  % unit vector of body frame expressed in inertial orientation



            frame_number = containers.Map({'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'},[1,2,3,4,5,6,7,8,9,10,11,12,13]);
            
            w_H_link=robot.get_frame_H(frame); % 4X4
            symmetric_axis=obj.set_symmetric_axis();
            w_kaxis_link=w_H_link(1:3,1:3)*symmetric_axis(1:3,frame_number(frame));
           
           
        end
        
        function symmetric_axis=set_symmetric_axis(obj)
            symmetric_axis=zeros(3,obj.N_link);
            symmetric_axis(1:3,1)=-[0;1;0]; %head -y
            symmetric_axis(1:3,2)=-[0;1;0];%chest  -y
            symmetric_axis(1:3,3)=-[0;0;1];%root link  -z
            symmetric_axis(1:3,4)=[0;0;1];%r_upper_arm  z
            symmetric_axis(1:3,5)=[0;0;1];%l_upper_room  z

            symmetric_axis(1:3,6)=[0;0;1];%r_elbow_1_aero_frame z
            symmetric_axis(1:3,7)=[0;0;1];%l_elbow_1_aero_frame  z
            symmetric_axis(1:3,8)=-[0;0;1];%r_upper_leg  -z
            symmetric_axis(1:3,9)=-[0;0;1];%l_upper_leg  -z
            symmetric_axis(1:3,10)=-[0;0;1];%r_lower_leg  -z
            symmetric_axis(1:3,11)=-[0;0;1];%l_lower_leg  -z
            symmetric_axis(1:3,12)=[0;0;1];%r_foot  z
            symmetric_axis(1:3,13)=[0;0;1];%l_foot  z
        end
        
        function AoA=compute_AoA(obj,w_kaxis_link,relative_velocity) % the angle between relative velocity and -k axis is defined as angle of attack
            
            AoA=atan2(norm(cross(relative_velocity,-w_kaxis_link)),dot(relative_velocity,-w_kaxis_link));
            
        end
            
       
        
        
        
        
    end
end
