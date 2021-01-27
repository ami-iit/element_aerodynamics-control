classdef af < handle
    % the class af (aerodynamics forces) handles the computation of
    % generalized aerodynamics forces on specific links (' chest' in this case).
    %rho:air density
    
    properties 
        C_D;C_L;
        w_H_chest (4,4) double;
        v_wind; 
        J_chest_iDynTree;
        rho;
        gama;
        Ka_chest;
        AoA; % angle of attack
     
    end
    
    methods
        function obj=af(v_wind,rho,gama)
            %af Construct an instance of this class
            %   Detailed explanation goes here
            obj.v_wind=v_wind;
            obj.rho=rho;
            obj.gama=gama;
            obj.Ka_chest=rho*gama*0.5;
        end
        
        function generalized_aerodynamics_force=compute_af(obj,robot,base_pose_dot, s_dot)
            % calculate generalized aerodynamics forces acting on robot
            % model
            
            
            aerodynamics_forces=obj.compute_af_link(Ka_chest,relative_air_velocity,C_D,C_L,AoA,k_axis);
            
        
        
        
        
        
        
        
        
        
        
    end
end
