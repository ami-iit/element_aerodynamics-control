%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci, Gabriele Nava
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function QP_flyingTorqueControl(block)

    setup(block);

    function setup(block)
    
        block.NumInputPorts  = 6; 
        block.NumOutputPorts = 2; 

        % Setup port properties to be dynamic
        block.SetPreCompInpPortInfoToDynamic;
        block.SetPreCompOutPortInfoToDynamic;

        % Override output port properties
        block.OutputPort(1).Dimensions = block.DialogPrm(1).Data; % jointTorques_star
        block.OutputPort(2).Dimensions = 1;                       % exitFlag_QP       

        for i = 1:block.NumInputPorts
            block.InputPort(i).DatatypeID        = -1; % 'inherited', see http://www.mathworks.com/help/simulink/slref/simulink.blockdata.html#f29-108672
            block.InputPort(i).Complexity        = 'Real';
            block.InputPort(i).DirectFeedthrough = true;
        end

        for i = 1:block.NumOutputPorts
            block.OutputPort(i).DatatypeID  = 0; % double
            block.OutputPort(i).Complexity  = 'Real';
        end

        % Register parameters
        block.NumDialogPrms      = 1;

        % Register sample times
        %  [0 offset]            : Continuous sample time
        %  [positive_num offset] : Discrete sample time
        %
        %  [-1, 0]               : Inherited sample time
        %  [-2, 0]               : Variable sample time
        block.SampleTimes        = [-1 0];

        % Specify the block simStateCompliance. The allowed values are:
        %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
        %    'DefaultSimState', < Same sim state as a built-in block
        %    'HasNoSimState',   < No sim state
        %    'CustomSimState',  < Has GetSimState and SetSimState methods
        %    'DisallowSimState' < Error out when saving or restoring the model sim state
        block.SimStateCompliance = 'DefaultSimState';

        %% ----------------------------------------------------------------
        %% The MATLAB S-function uses an internal registry for all
        %% block methods. You should register all relevant methods
        %% (optional and required) as illustrated below. You may choose
        %% any suitable name for the methods and implement these methods
        %% as local functions within the same file. See comments
        %% provided for each function for more information.
        %% ----------------------------------------------------------------
        block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
        block.RegBlockMethod('Outputs', @Outputs);     % Required
        block.RegBlockMethod('Terminate', @Terminate); % Required    
        block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDimensions);
        block.RegBlockMethod('SetOutputPortDimensions',@SetOutputPortDimensions);
    end

    function SetInputPortSamplingMode(block, idx, fd)

        block.InputPort(idx).SamplingMode = fd;

        for i=1:block.NumOutputPorts
            block.OutputPort(i).SamplingMode = fd;
        end
    end

    %% InitializeConditions:
    %%   Functionality    : Called at the start of simulation and if it is 
    %%                      present in an enabled subsystem configured to reset 
    %%                      states, it will be called when the enabled subsystem
    %%                      restarts execution to reset the states.
    %%   Required         : No
    %%   C-MEX counterpart: mdlInitializeConditions
    %%
    % function InitializeConditions(block)

    % end InitializeConditions

    %% Start:
    %%   Functionality    : Called once at start of model execution. If you
    %%                      have states that should be initialized once, this 
    %%                      is the place to do it.
    %%   Required         : No
    %%   C-MEX counterpart: mdlStart
    %%
    % function Start(block)

    % block.Dwork(1).Data = 0;

    % end function

    %% Outputs:
    %%   Functionality    : Called to generate block outputs in
    %%                      simulation step
    %%   Required         : Yes
    %%   C-MEX counterpart: mdlOutputs
    %%
    function Outputs(block)    
   
        % What follows aims at defining the hessian matrix H, the bias
        % vector g, and the constraint matrix A for the formalism of qpOases,ie
        %
        %   min (1/2) transpose(u) * H * u + transpose(u) * g
        %    s.t.
        %        lbA < A * u < ubA
        %
        % For further information, see
        % 
        % http://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
        %
        % [u,fval,exitflag,iter,lambda,auxOutput] = qpOASES(H,g,A,lb,ub,lbA,ubA{,options{,auxInput}})
        %  
        HessianMatrixQP            = block.InputPort(1).Data;
        gVectorQP                  = block.InputPort(2).Data;
        InequalityConstrMatrix     = block.InputPort(3).Data; 
        biasVectorInequalityConstr = block.InputPort(4).Data;
        feetContactIsActive        = block.InputPort(5).Data;
        jointTorquesSaturation     = block.InputPort(6).Data;
        ndof                       = block.DialogPrm(1).Data;
        
        % switch from balancing controller to flying controller
        if feetContactIsActive
               
           H    =  HessianMatrixQP;
           g    =  gVectorQP;
           ub   =  [ 1e14 * ones(12,1);  jointTorquesSaturation];
           lb   =  [-1e14 * ones(12,1); -jointTorquesSaturation];
           Aeq  =  [InequalityConstrMatrix, zeros(size(InequalityConstrMatrix, 1), ndof)];
           lbEq = -1e14 * ones(size(InequalityConstrMatrix, 1), 1); 
           ubEq =  biasVectorInequalityConstr;  
              
           [u, ~, exitFlagQP, ~, ~, ~] = qpOASES(H, g, Aeq, lb, ub, lbEq, ubEq);  
           block.OutputPort(1).Data    = u(13:end); % jointTorques_star
           
        else 
           % no contact forces (flying)
           H  =  HessianMatrixQP(13:end, 13:end);
           g  =  gVectorQP(13:end);
           ub =  jointTorquesSaturation;
           lb = -jointTorquesSaturation;
           
           [u, ~, exitFlagQP, ~, ~, ~]   = qpOASES(H, g, lb, ub);    
           block.OutputPort(1).Data      = u; % jointTorques_star
        end

        block.OutputPort(2).Data = exitFlagQP; % exitFlagQP    
    end

    function Terminate(~)
    
    end

    function SetOutputPortDimensions(s, port, dimsInfo) 
        s.OutputPort(port).Dimensions = dimsInfo;
    end

    function SetInputPortDimensions(s, port, dimsInfo)   
        s.InputPort(port).Dimensions = dimsInfo;
    end
end