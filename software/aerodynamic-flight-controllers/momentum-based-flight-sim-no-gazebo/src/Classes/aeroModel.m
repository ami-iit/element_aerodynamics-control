classdef aeroModel < handle
    % aeroModel A class representing the single link aerodynamic
    % characteristics

    properties
        frameNames; sphereModel; cylinderModel;
        C_D_sphere; C_N_sphere; 
        C_D_cylinder; C_N_cylinder; C_N_bar_cylinder; 
    end

    methods

        function obj = aeroModel(model_config)
            %JET Construct an instance of this class
            %   Detailed explanation goes here
            obj.frameNames    = model_config.framenames;
            obj.sphereModel   = model_config.sphereModel;
            obj.cylinderModel = model_config.cylinderModel;
        end

        function [C_D_sphere, C_N_sphere] = get_sphere_force_coefficients(obj, reynoldsNumber)
            % returns the spherical link aerodynamic drag coefficient

            if reynoldsNumber >= 0 && reynoldsNumber < 10
                C_D_sphere = 4.1275;
            elseif reynoldsNumber >= 10 && reynoldsNumber <= 2e4
                C_D_sphere = 24/reynoldsNumber * (1 + 0.150*reynoldsNumber^0.681) + ...
                    0.407/(1 + 8710/reynoldsNumber);
            elseif reynoldsNumber > 2e4 && reynoldsNumber < 6e6
                C_D_sphere  = interp1(Re_exp,Cd_exp,reynoldsNumber,'pchip');    %
            elseif reynoldsNumber >= 6e6
                C_D_sphere  = 0.188;
            end
            
            C_N_sphere = 0;

            obj.CD_sphere = C_D_sphere;
            obj.CL_sphere = C_L_sphere;
        end
        
        function [C_D_cylinder, C_N_cylinder, C_N_bar_cylinder] = get_cylinder_force_coefficients(obj, reynoldsNumber, aspectRatio, angleOfAttack)
            % returns the cylindrical link aerodynamic drag coefficient
            
            Cd_90    = interp1(obj.cylinderModel.Re_exp, obj.cylinderModel.Cd_exp, reynoldsNumber, 'pchip');
            ar_coeff = interp1(obj.cylinderModel.AR_exp, obj.cylinderModel.Cd_AR_factor, 1/aspectRatio, 'pchip');
            Cd_90_ar = Cd_90*ar_coeff;
            Cd_0     = interp1(obj.cylinderModel.AR_exp_ax, obj.cylinderModel.Cd_AR_ax_factor, aspectRatio, 'pchip');
            Cd_0     = Cd_0 * (pi/4) / aspectRatio;                        % correction for the ref. areas
            Re_ref   = 10^5;                                               % Re at which the experiment has been performed
            Cd_ref   = interp1(obj.cylinderModel.Re_exp, obj.cylinderModel.Cd_exp, Re_ref);
            Re_coeff = Cd_90/Cd_ref;
            Cd_0_Re  = Cd_0 * Re_coeff;

            C_D_cylinder = Cd_0_Re + (Cd_90_ar - Cd_0_Re) * abs(sind(angleOfAttack))^3;
            C_N_cylinder = Cd_90_ar * sind(angleOfAttack)^2 * cosd(angleOfAttack);

            % Corrected coefficient accounting for the cross product normalization term
            C_N_bar_cylinder = Cd_90_ar * sind(angleOfAttack) * cosd(angleOfAttack);
            
            obj.C_D_cylinder = C_D_cylinder;
            obj.C_N_cylinder = C_N_cylinder;
            obj.C_N_bar_cylinder = C_N_bar_cylinder;

        end


    end


end
