classdef aeroDyn < matlab.System & matlab.system.mixin.Propagates
    % step_block This block takes as input the joint torques and the
    % applied external forces and evolves the state of the robot
    
    % Public, tunable properties
    properties (Nontunable)
        aero_config;
        robot_config;
    end
    
    properties (DiscreteState)
        
    end
    
    properties (Access = private)
        aeroCoeff;
    end
    
    methods (Access = protected)
        
        function setupImpl(obj)
            obj.models = aeroModel(obj.aero_config);
            obj.robot  = Robot(obj.robot_config);
        end
        
        function [aerodynamicForces, generalizedAerodynamicWrench] = stepImpl(obj, basePose, baseVelocity, windSpeed)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            obj.conditions = set_global_aerodynamic_conditions(obj, windSpeed, baseVelocity);
            [aerodynamicForces, generalizedAerodynamicWrench] = obj.compute_aerodynamic_forces_and_generalized_aerodynamic_wrench(jet_input, basePose);
        end
        
        function [aerodynamicForces, generalizedAerodynamicWrench] = compute_aerodynamic_forces_and_generalized_aerodynamic_wrench(obj, baseVelocity, jointVelocities, basePose)
            aerodynamicForces = obj.compute_aerodynamic_forces_in_wrld_frame(baseVelocity, jointVelocities);
            aerodynamicWrenches = [aerodynamicForces; zeros(3, length(obj.models.frameNames))];
            generalizedAerodynamicWrench = obj.compute_generalized_aerodynamic_wrench(obj, aerodynamicWrenches);        
        end
        
        function aerodynamic_forces = compute_aerodynamic_forces_in_wrld_frame(obj, baseVelocity, jointVelocities)
                aerodynamic_forces = nan(3,length(obj.models.frameNames));
            for i = 1 : length(obj.models.frameNames)
                linkRelativeWindVelocity  = obj.compute_link_relative_wind_velocity(obj.models.frameNames{i}, baseVelocity, jointVelocities, obj.conditions.windSpeed);
                aerodynamic_forces(1:3,:) = obj.compute_link_aerodynamic_force_in_world_frame(obj.models.frameNames{i}, obj.models.frameAxis(:,i), obj.models.linkDiameter(i), ...
                                                                                              obj.models.linkLength(i), obj.models.linkReferenceArea(i), linkRelativeWindVelocity);
            end
        end

        function  link_aerodynamic_force = compute_link_aerodynamic_force_in_world_frame(obj, frameName, frameAxis, linkDiameter, linkLength, linkReferenceArea, linkRelativeWindVelocity)          
            linkAspectRatio    = linkLength/linkDiameter;
            linkAxisVersor     = obj.get_link_aerodynamic_axis_in_world_frame(obj, frameName, frameAxis);
            linkAngleOfAttack  = acosd(abs((transpose(linkAxisVersor * linkRelativeWindVelocity)) / (norm(linkRelativeWindVelocity) + 1e-8))); % [deg]
            linkReynoldsNumber = (obj.conditions.airDensity * norm(linkRelativeWindVelocity) * linkDiameter) / airDynamicViscosity;
            if matches(frameName,'head')
                [Cd, Cn] = obj.models.get_sphere_force_coefficients(linkReynoldsNumber);
                linkNormalForce = 0.5 * obj.conditions.airDensity * linkReferenceArea * Cn * ...
                                  sign(transpose(linkAxisVersor) * linkRelativeWindVelocity) * ...
                                  cross(cross(linkRelativeWindVelocity,linkAxisVersor),linkRelativeWindVelocity);
            else
                [Cd, ~, Cn_sin] = obj.models.get_cylinder_force_coefficients(linkReynoldsNumber, linkAspectRatio, linkAngleOfAttack);
                linkNormalForce = 0.5 * obj.conditions.airDensity * linkReferenceArea * Cn_sin * ...
                                  sign(transpose(linkAxisVersor) * linkRelativeWindVelocity) * ...
                                  cross(cross(linkRelativeWindVelocity,linkAxisVersor),linkRelativeWindVelocity);
            end
            linkDragForce = - 0.5 * obj.conditions.airDensity * linkReferenceArea * norm(linkRelativeWindVelocity) * Cd * linkRelativeWindVelocity;
            link_aerodynamic_force = linkNormalForce + linkDragForce;
        end
        


        function linkRelativeWindVelocity  = obj.compute_link_relative_wind_velocity(frameName, baseVelocity, jointVelocities, windSpeed)
            J_link = obj.robot.get_frame_jacobian(frameName);
            linkVelocity = J_link * [baseVelocity; jointVelocities];
            linkRelativeWindVelocity = windSpeed - linkVelocity(1:3);
        end

        function linkAxisVersor = get_link_aerodynamic_axis_in_world_frame(obj, frameName, frameAxis)
            w_H_l          = obj.robot.get_frame_H(frameName);
            linkAxisVersor = w_H_l(1:3,1:3) * frameAxis;
        end

        function set_global_aerodynamic_conditions(obj, windSpeed, airDensity, baseVelocity)
            obj.conditions.windSpeed            = windSpeed;
            obj.conditions.relativeWindVelocity = windSpeed - baseVelocity(1:3);
            obj.conditions.airDensity           = airDensity;
        end
        
        function generalizedAerodynamicWrench = compute_generalized_aerodynamic_wrench(obj, aerodynamicWrenches)
            generalizedAerodynamicWrench = zeros(29,1);
            for i = 1 : length(obj.models.frameNames)
                linkGenAeroWrench = obj.compute_link_generalized_aerodynamic_wrench(obj.models.frameNames{i}, aerodynamicWrenches(:, i));
                generalizedAerodynamicWrench = generalizedAerodynamicWrench + linkGenAeroWrench;
            end
        end

        function linkGenAeroWrench = compute_link_generalized_aerodynamic_wrench(obj, frameName, aerodynamicWrench)
            J_link = obj.robot.get_frame_jacobian(frameName);
            linkGenAeroWrench = J_link' * aerodynamicWrench;
        end
        
        function resetImpl(obj)
            
        end
        
        function [out, out2] = getOutputSizeImpl(~)
            % Return size for each output port
            out = [3 length(obj.models.frameNames)]; % aerodynamic forces
            out2 = [29 length(obj.models.frameNames)]; % generalized aerodynamic wrenches
        end
        
        function [out, out2] = getOutputDataTypeImpl(~)
            % Return data type for each output port
            out = "double";
            out2 = "double";
        end
        
        function [out, out2] = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            out = false;
            out2 = false;
        end
        
        function [out, out2] = isOutputFixedSizeImpl(~)
            % Return true for each output port with fixed size
            out = true;
            out2 = true;
        end
        
        
    end
    
end
