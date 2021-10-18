classdef Aerodynamics_force_link < handle
    % the class Aaerodynamics_forces handles the computation of
    % aerodynamic force on the whole robot and required parameters
    %  
    %rho:air density 1.225 kg/m^3 , at 101.325kPa and 15 degree
    %gama : shape coefficient
    %Ka : Ka = rho*gama/2
    %C_D_link: drag coefficient
    %C_N_link: normal force coefficient
    %com: center of mass
    %v_wind: wind velocity
    %@author: HUI TONG
    
%     properties
%         N_link;
%     end
    
    properties (Access = private)
        
        v_wind (3,1) double ; % R^3
        
        Ka double; 
        
        C_0;
        C_1;
        C_2;
        C_3;
        C_4;
        C_5;
    end
     
    
    
    methods  
        function obj=Aerodynamics_force_link(aerodynamics_config)
            % Construct an instance of this class
           
            %obj.N_link=aerodynamics_config.NOL;
            
            
            obj.Ka=aerodynamics_config.Ka;
            
            %%identified coefficients for force model
            %%cd=c0+c1*sin(a)^2*cos(b)^3+c2*cos(b)^3+c3*cos(b)^2
            %%cn=c4+c5*sin(2a)*cos(b)^2
            obj.C_0=aerodynamics_config.C_0;
            obj.C_1=aerodynamics_config.C_1;
            obj.C_2=aerodynamics_config.C_2;
            obj.C_3=aerodynamics_config.C_3;
            obj.C_4=aerodynamics_config.C_4;
            obj.C_5=aerodynamics_config.C_5;
        end
        
        
        
        function [C_D,C_N]=get_coeff(obj,alpha,beta)
            % get drag and normal force coefficients C_D and C_L
            
            %aerodynamic coefficients model
            C_D=obj.C_0+obj.C_1*(sin(alpha)^2)*(cos(beta)^3)+obj.C_2*(cos(beta)^3)+obj.C_3*(cos(beta)^2); % drag force coefficient
            C_N=obj.C_4+obj.C_5*sin(2*alpha)*(cos(beta)^2); % normal force coefficient
            
        end
        
       
        
        function relative_velocity=get_com_va(obj,robot,base_pose_dot,s_dot,v_wind) 
            % get relative velocity between the CoM velocity of the robot and the wind
           
            robot_velocity=[base_pose_dot;s_dot]; %(Ndof+6)X1
            
            J_com=robot.get_com_jacobian(); % 3X29 CoM jacobian
            linear_velocity_com=J_com*robot_velocity;% 3X1 CoM velocity (linear only) w.r.t inertial frame
          
            relative_velocity=linear_velocity_com-v_wind; % relative velocity expressed in world coordinate 
          
        end
        

        function alpha=compute_alpha(obj,w_kaxis_cfd,relative_velocity) 
            % the overall alpha angle of robot is defined as the angle between relative velocity and -k axis of body frame
            
            alpha=atan2(norm(cross(relative_velocity,-w_kaxis_cfd)),dot(relative_velocity,-w_kaxis_cfd));
            
        end
           
        
        function beta=compute_beta(obj,w_kaxis_cfd,w_iaxis_cfd,relative_velocity) %beta range [0,2pi]
            % the overall beta angle of robot is defined as the angle between the
            % projection of the relative velocity on plane (i,j) of defined body frame and i
            % axis of the body frame
            
            %calculate the projection of relative velocity (Va_proj) on the plane perpendicular to k
            %axis which is the plane of (i,j) of body frame
            Va_proj=relative_velocity-dot(relative_velocity,w_kaxis_cfd)/(norm(w_kaxis_cfd)^2)*(w_kaxis_cfd);
            sgn=sign(cross(w_iaxis_cfd,Va_proj));%sign of beta angle
            
            beta_nosign=atan2(norm(cross(Va_proj,w_iaxis_cfd)),dot(Va_proj,w_iaxis_cfd)); %angle value range [0,pi]
            
            
            %expand the range of beta into [0, 2pi]
            if sgn(3)<0
                beta=2*pi-beta_nosign;
            else
                beta=beta_nosign;
            end
            
            
        end
       
      
        function [w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd]=cfd_body_frame(obj,robot)
             %define body frame expressed in inertial frame
             
             % set the k axis direction
             w_H_head=robot.get_frame_H('head');
             o_head=w_H_head(1:3,4); % origin of head frame
             w_H_root=robot.get_frame_H('root_link');
             o_root=w_H_root(1:3,4); %origin of root link frame
             w_kaxis_cfd=(o_root-o_head)/norm(o_root-o_head); % kaxis
             
             % define the j axis direction, j axis is along the direction of
             % projectoion of the vector from right chest turbine to left
             % chest turbine on the plane that is perpendicular to k axis
             w_H_lchest=robot.get_frame_H('chest_l_jet_turbine');
             o_l=w_H_lchest(1:3,4); %origin of left chest turbine frame
            

             w_H_rchest=robot.get_frame_H('chest_r_jet_turbine');
             o_r=w_H_rchest(1:3,4); %origin of right chest turbine frame
             o_r_to_l=o_l-o_r; % vector starting from o_r and point towards o_l
             proj=o_r_to_l-dot(o_r_to_l,w_kaxis_cfd)/(norm(w_kaxis_cfd)^2)*(w_kaxis_cfd);% projection of vector o_r_to_l 
             %on the plane that is normal to vector w_kaxis_cfd
             
             
             w_jaxis_cfd=proj/norm(proj); % j axis is defined
             
             
             w_iaxis_cfd=cross(w_jaxis_cfd,w_kaxis_cfd); % i axis is defined according to right hand rule
      
        end
      
        function [w_xaxis_va,w_yaxis_va,w_zaxis_va]=cfd_velocity_frame(obj,w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd,beta,alpha)
            %compute the relative velocity frame unit vectors expressed in inertial frame
            
            %initial relative velocity frame (X0,Y0,Z0) before rotating, unit vectors are
            %expressed w.r.t inertial frame
            X0=w_jaxis_cfd;
            Y0=w_iaxis_cfd;
            Z0=-w_kaxis_cfd;
            w_R0=[X0,Y0,Z0]; % rotation matrix of initial velocity frame w.r.t the inertial frame I
            
            %the initial frame is rotated by (180-beta) degree around Z0 axis first which leads to frame (X1,Y1,Z1) where Z1=Z0, then it is
            %rotated by (alpha-90) degree around X1 axis which leads to (X2,Y2,Z2)
            %and Va ia along -Y2 direction
            
            %the above rotation can be seen as 'ZXZ' rotation with  angle1=
            %(180-beta), angle2=(alpha-90), angle3=0
            theta1=pi+beta;% beta \in (0,2pi)
            theta2=alpha-pi/2; % AoA \in (0,pi)
            theta3=0;
            
            %cos sin
            c1=cos(theta1); s1=sin(theta1);
            c2=cos(theta2); s2=sin(theta2);
            c3=cos(theta3); s3=sin(theta3);
            
            % rotation matrix of 'ZXZ' can be written as
            ZXZ=[c1*c3-c2*s1*s3  -c1*s3-c2*c3*s1  s1*s2;
                c3*s1+c1*c2*s3   c1*c2*c3-s1*s3   -c1*s2;
                s2*s3            c3*s2             c2];
            
            % matrix transformation: w_vector_va=w_R_0 * 0_R_va * va_vector_va  
            w_xaxis_va=w_R0*ZXZ*[1;0;0];
            w_yaxis_va=w_R0*ZXZ*[0;1;0]; % verify that it has the direction of -Va
            w_zaxis_va=w_R0*ZXZ*[0;0;1];
            
        end
      
        
        function [Fa_total,Fa_drag,Fa_side,Fa_normal]=compute_total_af(obj,w_kaxis_cfd,w_iaxis_cfd,w_xaxis_va,w_yaxis_va,w_zaxis_va,relative_velocity)
           %calculate total aerodynamic force which is decomposed into three
           %elements: drag force, normal force,side force in relative velocity
           %frame
            
           alpha=obj.compute_alpha(w_kaxis_cfd,relative_velocity);
           beta=obj.compute_beta(w_kaxis_cfd,w_iaxis_cfd,relative_velocity);
           [C_D,C_N]=obj.get_coeff(alpha,beta); % get coefficients of total aerodynamic force
          
           % aerodynamic force is decomposed into three components:drag (+Ya),
           % sideforce (+Xa),normal force (+Za)  , relative velocity is along -Ya
           % |Fd|=Ka*C_D*|Va|^2, |Fn|=Ka*C_N*|Va|^2, |Fs|=Ka*C_S*|Va|^2
           
           
           %drag force +Ya
           Fa_drag=obj.Ka*(norm(relative_velocity)^2)*C_D*w_yaxis_va; 
           
           %side force +Xa , assumed to be zero
           Fa_side=0*w_xaxis_va; %C_S=0
           
           %normal force +Za
           Fa_normal=obj.Ka*(norm(relative_velocity)^2)*C_N*w_zaxis_va;
           
           Fa_total=Fa_drag+Fa_side+Fa_normal;
        end
        
    end
end
