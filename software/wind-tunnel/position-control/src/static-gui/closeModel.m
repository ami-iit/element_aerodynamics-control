%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TEMPLATE FOR THE CLOSEMODEL FCN (TO BE USED IN THE STATIC GUI)

% Save and close the Simulink model through Matlab command line.
% It also closes the associate static GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('[closeModel]: closing the Simulink model...')

% save and close the Simulink model
save_system('WindTunnelControl.mdl');
close_system('WindTunnelControl.mdl');
rmpath(genpath('./src/'))

% close all figures
close all

disp('[closeModel]: done.')