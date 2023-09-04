% Author: Antonello Paolino
%
% August 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clc;

%% Initialization

% Data for the models
dataPath = './data/';
dataFile = [dataPath,'outputParameters.mat'];
load(dataFile);
jointConfigNames = fieldnames(data);

%% Aerodynamic forces application points definitions
aeroFrameNames = {'head', 'chest', 'chest_l_jet_turbine', 'chest_r_jet_turbine', ...
                  'l_upper_arm','l_arm_jet_turbine','r_upper_arm','r_arm_jet_turbine',...
                  'root_link','l_upper_leg','l_lower_leg','r_upper_leg','r_lower_leg'};

cfdLinkNames   = {'head', 'torso', 'left_back_turbine', 'right_back_turbine', ...
                  'left_arm','left_arm_turbine','right_arm','right_arm_turbine',...
                  'root_link','left_leg_upper','left_leg_lower','right_leg_upper','right_leg_lower'};

%% Load dataset
load([dataPath,'dataset.mat']);

%% Evaluate pre-computed models
% model coefficients
Cd_root_link_coefs = [0.0156; 0; 0.0362; -0.0353; 0];
Cd_arm_coefs       = [0.00172; 0.00167; 0.00575; 0; 0];
Cd_arm_turb_coefs  = [0.00559; 0.00150; 0.00769; 0; 0.00136];    

% model functions matrix
model_functions_matrix = @(alpha) [ ones(length(alpha),1) , ... 
                                    cosd(alpha)           , ...
                                    sind(alpha).^2        , ... 
                                    sind(alpha).^3        , ... 
                                    cosd(alpha).^3          ...
                                    ];

% Coefficients model
model = @(coefs,alpha) model_functions_matrix(alpha) * coefs;

% root_link
alpha_rl = linkAoAs_matrix(:,9);
Cd_root_link = model(Cd_root_link_coefs,alpha_rl);

% arms
alpha_la = linkAoAs_matrix(:,5);
Cd_left_arm = model(Cd_arm_coefs,alpha_la);
alpha_ra = linkAoAs_matrix(:,7);
Cd_right_arm = model(Cd_arm_coefs,alpha_ra);

% arm turbines
alpha_lat = linkAoAs_matrix(:,6);
Cd_left_arm_turb = model(Cd_arm_turb_coefs,alpha_lat);
alpha_rat = linkAoAs_matrix(:,8);
Cd_right_arm_turb = model(Cd_arm_turb_coefs,alpha_rat);


% Calculate partial coefficients
ironcubCd_partial = ironcubCdAs_full - Cd_root_link - Cd_left_arm - Cd_right_arm - Cd_left_arm_turb - Cd_right_arm_turb;


%% Generate links drag area ensemble aerodynamic model
% Initial guess from separated analysis
w_0 = [0; 0.0188; 0.0310; -0.0136; -0.0172;...
       0.0399; -0.00817; ...
       0.00956; -0.00319; 0.00302; 0.00461; ...
       0.00956; -0.00319; 0.00302; 0.00461; ...
       0.00172; 0.00167; 0.00575; ...
       0.00559; 0.00150; 0.00769; 0.00136; ...
       0.00172; 0.00167; 0.00575; ...
       0.00559; 0.00150; 0.00769; 0.00136; ...
       0; 0; 0; ...
       0; -0.00219; 0.0152;...
       0.00920; -0.00712; 0.0428; -0.0276; 0.00242; ...
       0; -0.00219; 0.0152;...
       0.00920; -0.00712; 0.0428; -0.0276; 0.00242 ...
       ];

% Build single link model matrices
head_X      = @(alpha_h)  [ones(length(alpha_h),1), cosd(alpha_h), sind(alpha_h).^2, sind(alpha_h).^3, cosd(alpha_h).^3];
torso_X     = @(alpha_t)  [ones(length(alpha_t),1), sind(alpha_t).^2];
back_turb_X = @(alpha_bt) [ones(length(alpha_bt),1), cosd(alpha_bt), sind(alpha_bt).^2, cosd(alpha_bt).^3];
arm_X       = @(alpha_a)  [ones(length(alpha_a),1), cosd(alpha_a), sind(alpha_a).^2];
arm_turb_X  = @(alpha_at) [ones(length(alpha_at),1), cosd(alpha_at), sind(alpha_at).^2, cosd(alpha_at).^3];
root_link_X = @(alpha_rl) [ones(length(alpha_rl),1), sind(alpha_rl).^2, sind(alpha_rl).^3];
upper_leg_X = @(alpha_ul) [ones(length(alpha_ul),1), cosd(alpha_ul), sind(alpha_ul).^2];
lower_leg_X = @(alpha_ll) [ones(length(alpha_ll),1), cosd(alpha_ll), sind(alpha_ll).^2, sind(alpha_ll).^3, cosd(alpha_ll).^3];

% Build robot full models matrix and separated links matrix
robot_X = @(alphas) ...
                [ head_X(alphas(:,1)), torso_X(alphas(:,2)), back_turb_X(alphas(:,3)), back_turb_X(alphas(:,4)), ...
                  arm_X(alphas(:,5)), arm_turb_X(alphas(:,6)), arm_X(alphas(:,7)),  arm_turb_X(alphas(:,8)), ...
                  root_link_X(alphas(:,9)), ... 
                  upper_leg_X(alphas(:,10)), lower_leg_X(alphas(:,11)), upper_leg_X(alphas(:,12)), lower_leg_X(alphas(:,13)) ];

links_X = @(alphas) ...
                blkdiag( head_X(alphas(:,1)), torso_X(alphas(:,2)), back_turb_X(alphas(:,3)), back_turb_X(alphas(:,4)), ...
                  arm_X(alphas(:,5)), arm_turb_X(alphas(:,6)), arm_X(alphas(:,7)),  arm_turb_X(alphas(:,8)), ...
                  root_link_X(alphas(:,9)), ... 
                  upper_leg_X(alphas(:,10)), lower_leg_X(alphas(:,11)), ...
                  upper_leg_X(alphas(:,12)), lower_leg_X(alphas(:,13)) );

% Evaluate block matrices 
X1 = robot_X(linkAoAs_matrix);
X2 = links_X(linkAoAs_matrix);

% Assemble X final matrix
X = [X1; X2; eye(length(X1(1,:)))];

% Evaluate related Y values
robot_Y = ironcubCd_partial;    % ironcubCd_partial || ironcubCdAs_full
links_Y = reshape(linkCdAs_matrix,[],1);

% Assemble Y matrix
Y = [robot_Y; links_Y; w_0];

% Find X matrix starting indices for each link
fake_X_mat = robot_X(10*ones(1,length(linkAoAs_matrix(1,:))));
start_in = find(fake_X_mat==1); %[1,5,7,10,13,16,20,23,27,30,32,37,39]; %find(fake_X_mat==1);
end_in   = [start_in(2:end)-1 length(fake_X_mat)];

% Equality constraints for symmetry
Aeq_back_turbs = Aeq_part_eq_init(   start_in([3 4]),   end_in([3 4]), length(X(1,:)) ); % back turbines constraints
Aeq_arms       = Aeq_part_eq_init(   start_in([5 7]),   end_in([5 7]), length(X(1,:)) ); % arms constraints
Aeq_arm_turbs  = Aeq_part_eq_init(   start_in([6 8]),   end_in([6 8]), length(X(1,:)) ); % arm turbines constraints
Aeq_upper_legs = Aeq_part_eq_init( start_in([10 12]), end_in([10 12]), length(X(1,:)) ); % upper legs constraints
Aeq_lower_legs = Aeq_part_eq_init( start_in([11 13]), end_in([11 13]), length(X(1,:)) ); % lower legs constraints

% Equality constraint for zero root_link coefs
Aeq_root_link = zeros( end_in(9) - start_in(9) + 1, length(X(1,:)) );
Aeq_root_link(1:end,start_in(9):end_in(9)) = eye( end_in(9) - start_in(9) + 1 );
Aeq_left_arm = zeros( end_in(5) - start_in(5) + 1, length(X(1,:)) );
Aeq_left_arm(1:end,start_in(5):end_in(5)) = eye( end_in(5) - start_in(5) + 1 );
Aeq_left_arm_turb = zeros( end_in(6) - start_in(6) + 1, length(X(1,:)) );
Aeq_left_arm_turb(1:end,start_in(6):end_in(6)) = eye( end_in(6) - start_in(6) + 1 );

% Equality constraint full matrix
Aeq = [Aeq_back_turbs; ...
       Aeq_arms; ...
       Aeq_arm_turbs; ...
       Aeq_upper_legs; ...
       Aeq_lower_legs; ...
       Aeq_root_link; ...
       Aeq_left_arm; ...
       Aeq_left_arm_turb ...
       ];

% Equality constraint condition
beq = zeros(size(Aeq,1),1);

% Inequality constraint for CdA always positive 
alpha_vec = transpose(linspace(0,180,19));
A = - robot_X(repmat(alpha_vec,1,length(X(1,:))));
b = zeros(length(A(:,1)),1);

% % Lower bounds
tol = 1e-4;
lb = w_0 - tol;
ub = w_0 + tol;

% least square linear optimization
% options = optimoptions("lsqlin","Algorithm","active-set");
Cd_coefs = lsqlin(X,Y,A,b,Aeq,beq,[],[],w_0);

%% Model Plots

N_alpha    = 1801;
alpha_plot = transpose(linspace(0,180,N_alpha));

Cd_models   = links_X(repmat(alpha_plot,1,length(X(1,:)))) * Cd_coefs;    

plotIndex = 0;

for linkIndex = 1 : length(cfdLinkNames) 

    if ~contains(cfdLinkNames{linkIndex},"right")

        Cd_model = Cd_models( N_alpha*(linkIndex-1)+1 : N_alpha*linkIndex );
        plotIndex = plotIndex + 1; 
        
        if plotIndex <= 4
            % plot link CdAs vs AoA
            fig = figure(1);
            ax(plotIndex) = subplot(2,2,plotIndex);
        else
            fig = figure(2);
            ax(plotIndex) = subplot(2,2,plotIndex-4);
        end
        subplot(ax(plotIndex));
        scatter(linkAoAs_matrix(:,linkIndex),linkCdAs_matrix(:,linkIndex),[],linkSsAs_matrix(:,linkIndex)); hold on;
        plot(alpha_plot,Cd_model,'k-','LineWidth',2); hold on;
        xlabel('$\alpha_{link}$','Interpreter','latex');
        ylabel('$C_D A$','Interpreter','latex');
        title(cfdLinkNames{linkIndex},'Interpreter','none');
        xlim([0 180]);
        grid on;
        c = colorbar;
        c.Limits = [0 180];
        c.Label.Interpreter = 'latex';
        c.Label.String = '$\beta_{link}$';
        c.Label.Position = [3, 95, 0];
        c.Label.Rotation = 0;
        c.Label.FontSize = 12;
        
    end

end



%% functions

function Aeq_part_eq = Aeq_part_eq_init(start_indices, end_indices, col_num)
    
    first_start_index = start_indices(1);
    second_start_index = start_indices(2);

    first_end_index = end_indices(1);
    second_end_index = end_indices(2);
    
    % init equality constraint matrix
    Aeq_part_eq = zeros( first_end_index - first_start_index + 1, col_num );
    
    % assign values for equality constraints
    Aeq_part_eq( 1:end , first_start_index:first_end_index ) = ...
                    eye( first_end_index - first_start_index + 1);
    Aeq_part_eq( 1:end , second_start_index:second_end_index ) = ...
                    - eye( second_end_index - second_start_index + 1);

end
