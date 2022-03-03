classdef Aerodynamics_force < handle
    
    % the class Aerodynamics_forces handles the computation of
    % aerodynamic force on the whole robot and required parameters
    %  
    % rho :air density 1.225 kg/m^3 , at 101.325kPa and 15 degree
    % gamma : shape coefficient
    % Ka : Ka = rho*gamma/2
    % C_D_link: drag coefficient
    % C_N_link: normal force coefficient
    % CoM: center of mass
    % v_wind: wind velocity
    % @author: HUI TONG
    
%     properties
%         N_link;
%     end
    
    properties (Access = private)
        
        v_wind (3,1) double; % R^3  
        Ka  double; 
        C_0 double;
        C_1 double;
        C_2 double;
        C_3 double;
        C_4 double;
        C_5 double;
    end
     
    methods 
        
        function obj = Aerodynamics_force(aerodynamics_config)
            
            % Construct an instance of this class
           
            %obj.N_link=aerodynamics_config.NOL;
            
            obj.Ka = aerodynamics_config.Ka;
            
            % identified coefficients for force model
            %
            % cd = c0+c1*sin(a)^2*cos(b)^3+c2*cos(b)^3+c3*cos(b)^2
            % cn = c4+c5*sin(2a)*cos(b)^2
            %
            obj.C_0 = aerodynamics_config.C_0;
            obj.C_1 = aerodynamics_config.C_1;
            obj.C_2 = aerodynamics_config.C_2;
            obj.C_3 = aerodynamics_config.C_3;
            obj.C_4 = aerodynamics_config.C_4;
            obj.C_5 = aerodynamics_config.C_5;
        end
        
% ----------------------------------------------------------------------- %
        
        function [C_D, C_N] = get_coeff(obj, alpha, beta)
            
            % get drag and normal force coefficients C_D and C_L
            
            % aerodynamic coefficients models
            C_D = obj.C_0 + obj.C_1*(sin(alpha)^2)*(sin(beta)^2) + obj.C_2*(sin(alpha)^2) + obj.C_3*(sin(beta)^2);  % drag force coefficient
            C_N = obj.C_4 + obj.C_5*sin(2*alpha)*sin(alpha)^2*(cos(beta)^2);  % normal force coefficient
        end
        
        function relative_velocity = get_com_v_rel(obj, robot, base_pose_dot, s_dot, v_wind) 
            
            % get relative velocity between the CoM velocity of the robot and the wind
           
            state_velocity = [base_pose_dot; s_dot]; % (Ndof+6)x1
            
            J_com = robot.get_com_jacobian(); % 3x29 CoM jacobian
            
            linear_velocity_com = J_com*state_velocity; % 3x1 CoM velocity (linear only) w.r.t inertial frame
          
            relative_velocity = linear_velocity_com - v_wind; % relative velocity expressed in world coordinate 
        end
        
        function alpha = compute_alpha(obj, k_axis_cfd, relative_velocity)
            
            % the overall alpha angle of robot is defined as the angle between relative velocity and -k axis of body frame
            alpha = atan2(norm(cross(relative_velocity, -k_axis_cfd)), dot(relative_velocity, -k_axis_cfd));          
        end
           
        function beta = compute_beta(obj, k_axis_cfd, i_axis_cfd, relative_velocity) 
            
            % beta range [0, 2pi]
            
            % the overall beta angle of robot is defined as the angle between the
            % projection of the relative velocity on plane (i,j) of defined body frame and i
            % axis of the body frame
            
            % calculate the projection of relative velocity (Va_proj) on the plane perpendicular to k
            % axis which is the plane of (i,j) of body frame
            %
            Va_proj = relative_velocity - dot(relative_velocity, k_axis_cfd)/(norm(k_axis_cfd)^2)*(k_axis_cfd);
            sgn     = sign(cross(i_axis_cfd, Va_proj)); % sign of beta angle
            
            beta_unsigned = atan2(norm(cross(Va_proj, i_axis_cfd)), dot(Va_proj, i_axis_cfd)); % angle value range [0,pi]
            
            % expand the range of beta into [0, 2pi]
            if sgn(3) < 0
                
                beta = 2*pi - beta_unsigned;
            else
                beta = beta_unsigned;
            end     
        end   
      
        function [k_axis_cfd, i_axis_cfd, j_axis_cfd] = compute_body_frame(obj, robot)
            
             % define body frame expressed in inertial frame
             
             % set the k axis direction
             w_H_head = robot.get_frame_H('head');
             w_o_head = w_H_head(1:3,4); % origin of head frame
             
             w_H_root = robot.get_frame_H('root_link');
             w_o_root = w_H_root(1:3,4); % origin of root link frame
             
             k_axis_cfd = (w_o_root - w_o_head)/norm(w_o_root - w_o_head); % k axis
             
             % define the j axis direction, j axis is along the direction of
             % projection of the vector from right chest turbine to left
             % chest turbine on the plane that is perpendicular to k axis
             %
             w_H_lchest = robot.get_frame_H('chest_l_jet_turbine');
             w_o_lchest = w_H_lchest(1:3,4); % origin of left chest turbine frame 
             w_H_rchest = robot.get_frame_H('chest_r_jet_turbine');
             w_o_rchest = w_H_rchest(1:3,4); % origin of right chest turbine frame
             
             % projection on the plane that is normal to vector k_axis_cfd
             d_lchest_rchest_ij = (w_o_lchest-w_o_rchest)-dot((w_o_lchest-w_o_rchest), k_axis_cfd)/(norm(k_axis_cfd)^2)*(k_axis_cfd);
             
             i_axis_cfd = d_lchest_rchest_ij/norm(d_lchest_rchest_ij); % j axis is defined
             j_axis_cfd = cross(k_axis_cfd, i_axis_cfd);               % i axis is defined according to right hand rule
        end
      
        function [x_axis_va, y_axis_va, z_axis_va] = compute_velocity_frame(obj, k_axis_cfd, i_axis_cfd, j_axis_cfd, beta, alpha)
          
            % compute the relative velocity frame unit vectors expressed in inertial frame
            
            % initial relative velocity frame (X0,Y0,Z0) before rotating, unit vectors are
            % expressed w.r.t inertial frame
            x0 =  j_axis_cfd;
            y0 =  i_axis_cfd;
            z0 = -k_axis_cfd;
            
            w_R_0 = [x0, y0, z0]; % rotation matrix of initial velocity frame w.r.t the inertial frame I
            
            % the initial frame is rotated by (180-beta) degree around z0 axis first 
            % which leads to frame (x1,y1,z1) where z1 = z0, then it is
            % rotated by (alpha-90) degree around x1 axis which leads to (x2,y2,z2)
            % and Va along -y2 direction
            
            % the above rotation can be seen as 'zxz' rotation with  
            % angle1 = (180 - beta), angle2 = (alpha - 90), angle3 = 0
            %
            theta1 = pi+beta; % beta \in (0,2pi)
            theta2 = alpha-pi/2; % AoA \in (0,pi)
            theta3 = 0;
            
            % cos sin
            c1 = cos(theta1); s1 = sin(theta1);
            c2 = cos(theta2); s2 = sin(theta2);
            c3 = cos(theta3); s3 = sin(theta3);
            
            % rotation matrix of 'ZXZ' can be written as
            O_R_va = [c1*c3-c2*s1*s3  -c1*s3-c2*c3*s1   s1*s2;
                      c3*s1+c1*c2*s3   c1*c2*c3-s1*s3  -c1*s2;
                      s2*s3            c3*s2            c2];
            
            % matrix transformation: w_vector_va = w_R_0 * 0_R_va * va_vector_va  
            x_axis_va = w_R_0 * O_R_va*[1;0;0];
            y_axis_va = w_R_0 * O_R_va*[0;1;0]; % verify that it has the direction of -Va
            z_axis_va = w_R_0 * O_R_va*[0;0;1];
        end
        
        function [Fa_total, Fa_drag, Fa_side, Fa_normal] = compute_aerodynamic_force(obj, k_axis_cfd, i_axis_cfd, x_axis_va, y_axis_va, z_axis_va, relative_velocity)

            % calculate total aerodynamic force which is decomposed into three
            % elements: drag force, normal force, side force in relative velocity
            % frame
            %
            alpha      = obj.compute_alpha(k_axis_cfd, relative_velocity);
            beta       = obj.compute_beta(k_axis_cfd, i_axis_cfd, relative_velocity);
            [C_D, C_N] = obj.get_coeff(alpha, beta); % get coefficients of total aerodynamic force

            % aerodynamic force is decomposed into three components: drag (+Ya),
            % sideforce (+Xa), normal force (+Za), relative velocity is along -Ya
            % |Fd|=Ka*C_D*|Va|^2, |Fn|=Ka*C_N*|Va|^2, |Fs|=Ka*C_S*|Va|^2

            % drag force +Ya
            Fa_drag = obj.Ka*(norm(relative_velocity)^2)*C_D*y_axis_va;

            % side force +Xa , assumed to be zero
            Fa_side = 0*x_axis_va; % C_S = 0

            % normal force +Za
            Fa_normal = obj.Ka*(norm(relative_velocity)^2)*C_N*z_axis_va;

            Fa_total = Fa_drag + Fa_side + Fa_normal;
        end

        function controller_aerodynamic_forces = compute_controller_aerodynamic_force(obj, x_axis_va, y_axis_va, z_axis_va, relative_velocity, beta, alpha, config)

            % calculate the total aerodynamic force used by the controller
            % in the simulations, accounting for errors (both noise and
            % calibration) on the measures of alpha, beta and the module of
            % relative velocity
            
            % Apply calibration error on aerodynamics measures
            alpha_meas             = (1 + config.controller_sensors_calib_error) * alpha;
            beta_meas              = (1 + config.controller_sensors_calib_error) * beta;
            relative_velocity_meas = (1 + config.controller_sensors_calib_error) * relative_velocity;

            % Apply relative errors on aerodynamics measures
            alpha_meas             = (1 + config.controller_sensors_noise*(rand(1) - 0.5)) * alpha_meas;
            beta_meas              = (1 + config.controller_sensors_noise*(rand(1) - 0.5)) * beta_meas;
            relative_velocity_meas = (1 + config.controller_sensors_noise*(rand(1) - 0.5)) * relative_velocity_meas;

            

            % Get aerodynamic coefficients from the model
            [C_D, C_N] = obj.get_coeff(alpha_meas, beta_meas); % get coefficients of total aerodynamic force

            % aerodynamic force is decomposed into three components: drag (+Ya),
            % sideforce (+Xa), normal force (+Za), relative velocity is along -Ya
            % |Fd|=Ka*C_D*|Va|^2, |Fn|=Ka*C_N*|Va|^2, |Fs|=Ka*C_S*|Va|^2

            % drag force +Ya
            Fa_drag = obj.Ka*(norm(relative_velocity_meas)^2)*C_D*y_axis_va;

            % side force +Xa , assumed to be zero
            Fa_side = 0*x_axis_va; % C_S = 0

            % normal force +Za
            Fa_normal = obj.Ka*(norm(relative_velocity_meas)^2)*C_N*z_axis_va;

            controller_aerodynamic_forces = Fa_drag + Fa_side + Fa_normal;

        end
    end
end
