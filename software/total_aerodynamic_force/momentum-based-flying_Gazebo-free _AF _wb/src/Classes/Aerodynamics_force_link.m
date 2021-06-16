classdef Aerodynamics_force_link < handle
    % the class Aaerodynamics_forces handles the computation of
    %  aerodynamics forces on one single link and total aerodynamic force
    %  on the whole robot
    %rho:air density
    %gama : shape coefficient
    %Ka : Ka = rho*gama/2
    %C_D_link: drag coefficient
    %C_N_link:normal force coefficient
    %link list:  ['head'=1,
    %,'chest'=2,'root_link'=3,'r_upper_arm'=4,'l_upper_arm'=5,'r_elbow_1_aero_frame'=6,
    %'l_elbow_1_aero_frame'=7,'r_upper_leg'=8,'l_upper_leg'=9,'r_lower_leg'=10,
    %'l_lower_leg'=11,'r_foot'=12,'l_foot'=13] 
    %extra frame 'com' for adding total aerodynamic force 'com'=14
    % author: HUI TONG
    
    properties
        N_link;
    end
    
    properties (Access = private)
        
        v_wind (3,1) double ; % R^3
        
        rho;%R
        gama (14,1) double;
        Ka (14,1) double; 
        
        C_0;
        C_1;
        C_2;
        C_3;
        C_4;
        C_5;
    end
     
    
    
    methods  
        function obj=Aerodynamics_force_link(aerodynamics_config)
            %af Construct an instance of this class
            %   v_wind - the wind velocity vector expressed in inertial frame 
            %   rho - air density
            %   gama - shape coefficient
            obj.N_link=aerodynamics_config.NOL;
            %obj.v_wind=aerodynamics_config.v_wind;
            %obj.v_wind=v_wind;
            
            obj.rho=aerodynamics_config.rho;
            obj.gama=aerodynamics_config.gama;%vector N_linkX1
            obj.Ka=aerodynamics_config.Ka;%vector N_linkX1
            
            %%identified coefficients for force model
            %%cd=c0+c1*sin(a)^2*cos(b)^2+c2*cos(b)^2
            %%cn=c3*sin(2a)
            obj.C_0=aerodynamics_config.C_0;
            obj.C_1=aerodynamics_config.C_1;
            obj.C_2=aerodynamics_config.C_2;
            obj.C_3=aerodynamics_config.C_3;
            obj.C_4=aerodynamics_config.C_4;
            obj.C_5=aerodynamics_config.C_5;
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
    
        
        
        function aerodynamics_forces=compute_af_link(obj,relative_velocity,w_kaxis_link,frame)
            
            % calculate aerodynamics forces (R^3) vector acting on each link expressed in world frame since all the
            % vectors in formula are expressed in world frame
            
           AoA=obj.compute_AoA(relative_velocity,w_kaxis_link);% compute angle of attack for a specific link
            
            [Ka_link,C_D_link,C_N_link]=obj.get_coeff(frame,AoA); % get coefficients of one single link
            
            AoA = max(AoA, 1e-8); %set tolerance to avoid Inf value 
             aerodynamics_forces=-Ka_link*norm(relative_velocity)*((C_D_link+C_N_link*cot(AoA))*relative_velocity+C_N_link/sin(AoA)*norm(relative_velocity)*w_kaxis_link);
            % aerodynamics_forces \in R^3
           
        end
        
        function [Ka_link,C_D_link,C_N_link]=get_coeff(obj,frame,AoA,beta) % get force coefficients and shape coefficient for each link
            
            frame_number = containers.Map({'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot','com'},[1,2,3,4,5,6,7,8,9,10,11,12,13,14]);
            %the frame 'com' is not a real frame which can be found in urdf
            %model, it is a virtual frame used to compute CoM position and
            %velocity
            
            Ka_link=obj.Ka(frame_number(frame));
            
            %identified force coefficients
            C_0_link=obj.C_0(frame_number(frame));
            C_1_link=obj.C_1(frame_number(frame));
            C_2_link=obj.C_2(frame_number(frame));
            C_3_link=obj.C_3(frame_number(frame));
            C_4_link=obj.C_4(frame_number(frame));
            C_5_link=obj.C_5(frame_number(frame));
            
            %aerodynamic coefficients model
            C_D_link=C_0_link+C_1_link*(sin(AoA)^2)*(cos(beta)^3)+C_2_link*(cos(beta)^3)+C_3_link*(cos(beta)^2); % drag force coefficient
            C_N_link=C_4_link+C_5_link*sin(2*AoA)*(cos(beta)^2); % normal force coefficient
            
        end
        
        function relative_velocity=compute_relative_v(obj,robot,base_pose_dot,s_dot,frame,v_wind) %base__pose_dot and s_dot are from the state before forward dynamics
            %this function computes the relative velocity between linear
            %velocity of link frame origin w.r.t inertial frame and the
            %wind velocity expressed in the inertial frame
            
            
            robot_velocity=[base_pose_dot;s_dot]; %(Ndof+6)X1
            J=robot.get_frame_jacobian(frame);% 6X(Ndof+6)
           
            link_velocity=J*robot_velocity;% 6X1 link velocity (linear and angular) w.r.t inertial frame
            linear_velocity_link=link_velocity(1:3);
           
            relative_velocity=linear_velocity_link-v_wind; %expressed in world coordinate 
          
        end
        
        function Va_com=get_com_va(obj,robot,base_pose_dot,s_dot,v_wind) % get relative velocity of the robot and CoM velocity is assumed to present the
            %robot linear velocity
             
            robot_velocity=[base_pose_dot;s_dot]; %(Ndof+6)X1
            
            J_com=robot.get_com_jacobian(); % 3X29 CoM jacobian
            linear_velocity_com=J_com*robot_velocity;% 3X1 CoM velocity (linear only) w.r.t inertial frame
          
            Va_com=linear_velocity_com-v_wind; % relative velocity expressed in world coordinate 
          
        end
        
        function w_kaxis_link=compute_kaxis(obj,robot,frame)  % unit vector k of body frame expressed in inertial orientation



            frame_number = containers.Map({'head','chest','root_link','r_upper_arm','l_upper_arm',...
                'r_elbow_1_aero_frame','l_elbow_1_aero_frame','r_upper_leg','l_upper_leg','r_lower_leg','l_lower_leg','r_foot','l_foot'},[1,2,3,4,5,6,7,8,9,10,11,12,13]);
            
            w_H_link=robot.get_frame_H(frame); % 4X4
            symmetric_axis=obj.set_symmetric_axis();
            w_kaxis_link=w_H_link(1:3,1:3)*symmetric_axis(1:3,frame_number(frame));
           
           
        end
        
        function symmetric_axis=set_symmetric_axis(obj) %select the symmetric axis of each link as the k axis of link frame from urdf frames
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
           
        
        function beta=compute_beta(obj,w_kaxis_cfd,w_iaxis_cfd,Va_com)
           % the overall beta angle of robot is defined as the angle between the
           % projection of Va_com on plane (i,j) of defined body frame and i
           % axis of the body frame
           
            %calculate the projection of va on the plane perpendicular to k
            %axis which is the plane of (i,j) of body frame
          Va_proj=Va_com-dot(Va_com,w_kaxis_cfd)/(norm(w_kaxis_cfd)^2)*(w_kaxis_cfd);
          sgn=sign(cross(w_iaxis_cfd,Va_proj));%sign of beta angle 
          
          beta_nosign=atan2(norm(cross(Va_proj,w_iaxis_cfd)),dot(Va_proj,w_iaxis_cfd)); %angle value range [0,pi]
          
          if sgn(3)<0
              beta=2*pi-beta_nosign;
          else
              beta=beta_nosign;
          end
          %beta range [0,2pi]
          
        end
       
      function [w_kaxis_root,w_iaxis_root]=root_rot(obj,robot)
             
             
            w_H_root=robot.get_frame_H('root_link'); % 4X4
            symmetric_axis=obj.set_symmetric_axis();
            w_kaxis_root=w_H_root(1:3,1:3)*symmetric_axis(1:3,2);
            
            w_iaxis_root=w_H_root(1:3,1:3)*[1;0;0];
      end  
      
      %instead of using the existed link frame, one extra frame is created to
      %present the body frame of robot
      function [w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd]=cfd_body_frame(obj,robot)
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
             o_l=w_H_lchest(1:3,4);
            

             w_H_rchest=robot.get_frame_H('chest_r_jet_turbine');
             o_r=w_H_rchest(1:3,4);
             o_r_to_l=o_l-o_r; % vector starting from o_r and point towards o_l
             proj=o_r_to_l-dot(o_r_to_l,w_kaxis_cfd)/(norm(w_kaxis_cfd)^2)*(w_kaxis_cfd);% projection of vector o_r_to_l 
             %on the plane that is normal to vector w_kaxis_cfd
             
             
             w_jaxis_cfd=proj/norm(proj); % j axis is defined
             
             
             w_iaxis_cfd=cross(w_jaxis_cfd,w_kaxis_cfd);
      
      end
      
      function [w_xaxis_va,w_yaxis_va,w_zaxis_va]=cfd_velocity_frame(obj,w_kaxis_cfd,w_iaxis_cfd,w_jaxis_cfd,beta,AoA)
       %compute the relative velocity frame unit vectors expressed in inertial frame
       
       %initial velocity frame (X0,Y0,Z0) before rotating, unit vectors are
       %expressed w.r.t inertial frame 
       X0=w_jaxis_cfd;
       Y0=w_iaxis_cfd;
       Z0=-w_kaxis_cfd;
       w_R0=[X0,Y0,Z0]; % rotation matrix of initial velocity frame w.r.t the inertial frame I 
       
       %the initial frame is rotated by 180-beta Z0 axis first which lead to frame (X1,Y1,Z1) where Z1=Z0, then it is
       %rotated by (alpha-90) degree around X1 axis which leads to (X2,Y2,Z2)
       %and Va ia along -Y2 direction
       
       %the above rotation can be seen as ZXZ rotation with  angle1=
       %(180-beta), angle2=(alpha-90), angle3=0 
       theta1=pi+beta;% beta \in (0,2pi)
       theta2=AoA-pi/2; % AoA \in (0,pi)
       theta3=0;
       
       %cos sin
       c1=cos(theta1); s1=sin(theta1);
       c2=cos(theta2); s2=sin(theta2);
       c3=cos(theta3); s3=sin(theta3);
       
       % rotation matrix of ZXZ can be written as 
       ZXZ=[c1*c3-c2*s1*s3  -c1*s3-c2*c3*s1  s1*s2;
           c3*s1+c1*c2*s3   c1*c2*c3-s1*s3   -c1*s2;
           s2*s3            c3*s2             c2];   
       
       % w_vector_va=w_R_0 * 0_R_va * va_vector_va
       w_xaxis_va=w_R0*ZXZ*[1;0;0];
       w_yaxis_va=w_R0*ZXZ*[0;1;0]; % verify that it has the direction of -Va
       w_zaxis_va=w_R0*ZXZ*[0;0;1];
            
      end
      
        %calculate total aerodynamic force which is decomposed into three
        %elements: drag force, normal force,side force in relative velocity
        %frame
        function [Fa_total,Fa_drag,Fa_side,Fa_normal]=compute_total_af(obj,w_kaxis_cfd,w_iaxis_cfd,w_xaxis_va,w_yaxis_va,w_zaxis_va,Va_com)
           
            %the overall angle of attack is defined as the angle between Va_com and -k
            %axis of chest frame
           AoA=obj.compute_AoA(w_kaxis_cfd,Va_com);
           beta=obj.compute_beta(w_kaxis_cfd,w_iaxis_cfd,Va_com);
           [Ka_com,C_D_tot,C_N_tot]=obj.get_coeff('com',AoA,beta); % get coefficients of total aerodynamic force
          
           % aerodynamic force is decomposed into three components:drag (+Ya),
           % sideforce (+Xa),normal force (+Za)  , Va_com is along -Ya
           % |Fa|=Ka*C_a*|Va|^2
           
           
           %drag force +Ya
           Fa_drag=Ka_com*(norm(Va_com)^2)*C_D_tot*w_yaxis_va; 
           
           %side force +Xa , assumed to be zero
           Fa_side=0*w_xaxis_va;
           
           %normal force +Za
           Fa_normal=Ka_com*(norm(Va_com)^2)*C_N_tot*w_zaxis_va;
           
           Fa_total=Fa_drag+Fa_side+Fa_normal;
        end
        
    end
end
