% Run this script to reproduce the simulations for the ICRA 2022 paper on
% aerodynamics control design.
%
clc
Config_ICRA.simulationType = {'hovering', 'high_speed'};
Config_ICRA.controllerType = {'baseline', 'gain_scheduling', 'f_linearization'};

for i=1:2
    
    for j=1:3
        
        currentSimType     = Config_ICRA.simulationType{i};
        currentControlType = Config_ICRA.controllerType{j};
        disp(['CURRENT_SIMULATION: ', currentSimType,'-',currentControlType])
        
        switch currentSimType
            
            case 'hovering'
                
                Config_ICRA.chosenSim = 2;
                
            case 'high_speed'
                
                Config_ICRA.chosenSim = 3;
                
            otherwise
                error('specified sim. type not valid!')
        end
        
        switch currentControlType
            
            case 'baseline'
                
                Config_ICRA.use_aerodynamics_forces_feedback = false;
                Config_ICRA.use_gain_scheduling = false;
                
            case 'gain_scheduling'
                
                Config_ICRA.use_aerodynamics_forces_feedback = false;
                Config_ICRA.use_gain_scheduling = true;
                
            case 'f_linearization'
                
                Config_ICRA.use_aerodynamics_forces_feedback = true;
                Config_ICRA.use_gain_scheduling = false;
        end
        
        Config_ICRA.folderName = [currentSimType,'_',currentControlType];
        sim('momentumBasedFlight.mdl');
    end
end
