function [sysest, x0] = lpvssest_(sysest, data, options)
%LPVSSEST_ Estimating an LPV-SS model from data    
%
%   sys = lpvssest_(data, init_est)
%      estimates an LPV-SS model from data using the input-scheduling-ouput
%      data DATA.
%
%   DATA is the time-domain estimation data captured in an LPVIDDATA
%      object. Type "help lpviddata" for more information.
%   INIT_EST is an initial LPV-SS model. This can either be an LPVSS or
%      LPVIDSS object.
%
%   sys = lpvssest_(data, init_est, options)
%   [sys, x0] = lpvssest_(__)
%
%   OPTIONS specifies the options of the estimation methodology. Use the
%      "lpvssestOptions" command to configure OPTIONS.
%
%   x0 is the estimated initial state.
%
%
% See also PMATRIX, LPVSSESTOPTIONS

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

% TODO: 
% - LPVSSEST_ cannot handle nonfree parameters yet.
% - Implement InputDelay
% - Estimate covariance on the parameters

    %% Input validation
    
    narginchk(2,3);
    
    if sysest.Ts == 0
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','discrete-time LPV SS representation','lpvidss/lpvssest_');
    end
    
    if isempty(sysest); x0 = []; return; end
    
    if ~isa(data, 'lpviddata')
        LPVcore.error('LPVcore:general:InvalidInputArgument','second','lpviddata object','lpvidss/lpvssest_');
    end
    
	[~,ny,np,nu] = size(data);

    if ~isequal(size(sysest),[ny nu]) || np ~= sysest.SchedulingDimension
        LPVcore.error('LPVcore:general:InvalidInput','Data and model object should have equivalent input, scheduling, and output dimension','lpvidss/lpvssest_');
    end
    
    if ny == 0
        LPVcore.error('LPVcore:general:InvalidInput','There is no output defined and, therefore, minimizing w.r.t. the output is not possible.','lpvidss/lpvssest_');
    end
    
    
    if nargin > 2
        if ~isa(options, 'lpvssestOptions')
            LPVcore.error('LPVcore:general:InvalidInputArgument','third','lpvssestOptions object','lpvidss/lpvssest_');
        end
    else
        options = lpvssestOptions;
    end

    
    if strcmp(options.SearchMethod,'em') && strcmp(typeNoiseModel(sysest),'innovation') && ~isempty(sysest.K)
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with general noise model','lpvidss/lpvssest_');
	elseif ~strcmp(options.SearchMethod,'em') && strcmp(typeNoiseModel(sysest),'general')
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with innovation noise model','lpvidss/lpvssest_');
    end
    
    
    if strcmp(typeNoiseModel(sysest),'general') && (ispmatrix(sysest.G) || ispmatrix(sysest.H))
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with parameter independent general noise model','lpvidss/lpvssest_');
    end
    
    
    if (isempty(sysest.A) && isempty(sysest.B) && isempty(sysest.K)) || isempty(sysest.C)
        LPVcore.error('LPVcore:general:InvalidInput','The initial LPV-SS model has either no state dynamics or the state is not observed in the output (C is empty).','lpvidss/lpvssest_');
    end
    

    %% Pre-processing: Remove trend
    data = rmOffset(data, options.OutputOffset,...
                       options.SchedulingOffset, options.InputOffset);
                
                   
	%% Estimation
    
    switch options.SearchMethod
        
        case {'fmincon','lsqnonlin'}
            
            sysest = lpvssest_min_matlab(sysest, data, options);
            
        case 'em'
            
            sysest = lpvssest_em(sysest, data, options);   
            
        otherwise % egn
            
            sysest = lpvssest_egn_(sysest, data, options);      
    end    
    
    
    x0 = sysest.X0;

end

%-------------------- LOCAL FUNCTIONS -------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sysest = lpvssest_min_matlab(sysest, data, options, DispWind)


    Disp = ~strcmpi(options.Display,'off');

        % Collect all essential information
    [sys, theta0] = LPVcore.numerics.lpvidss2struct(sysest,options);


    % Pre-compute the extended scheduling signal rho(t)
    if ~isempty(sysest.Structure_.SchedulingTimeMap)
        rho = createRho(sysest.Structure_.SchedulingTimeMap, data.SchedulingData); %, pinit, pterm);
    else
        rho = [];
    end


    W = options.OutputWeight;
    if isempty(W);  W = eye(sysest.IOSize(1));
    else
        LPVcore.error('LPVcore:general:InvalidOption','OutputWeight','lpvssestOptions','empty matrix for enhanced gauss newton method', 'lpvidss/lpvssest_');
    end

    
    if strcmpi(options.SearchMethod,'fmincon')
        
        if Disp && DispWind.ActiveJavaWindow
            idDisplayEstimationInfo('Progress',{' ',LPVcore.messageStr('LPVcore:lpvidss:tabHearderFmincon1'),LPVcore.messageStr('LPVcore:lpvidss:tabHearderFmincon2')},DispWind);
            options.SearchOptions.OutputFcn = {@(x,o,s)localOutFunFmincon(x,o,s,DispWind)};    % Display everything in the window
        end
        options.SearchOptions.Display = 'off';
        
        theta = fmincon(@(x) LPVcore.numerics.lpvssest_fmincon(x, data.InputData, rho, data.OutputData, W, sys), ...
                     theta0,[],[],[],[],[],[],[],options.SearchOptions);
        
    else
        
        if Disp && DispWind.ActiveJavaWindow
            idDisplayEstimationInfo('Progress',{' ',LPVcore.messageStr('LPVcore:lpvidss:tabHearderLsqnonlin1'),LPVcore.messageStr('LPVcore:lpvidss:tabHearderLsqnonlin2')},DispWind);
            options.SearchOptions.OutputFcn = {@(x,o,s)localOutFunLsqnonlin(x,o,s,DispWind)};    % Display everything in the window
        end 
        options.SearchOptions.Display = 'off';
        
        theta = lsqnonlin(@(x) LPVcore.numerics.lpvssest_lsqnonlin(x, data.InputData, rho, data.OutputData, W, sys), ...
                    theta0,[],[],options.SearchOptions);
                
    end
    
    %% Post-processing: Save data in the lpvidss object
    
	sys = LPVcore.numerics.theta2sys(theta,sys);
        
    if sys.isA == 2
        for i = 1:sys.npsiA
           sysest.Structure_.A.Parameter(i).Value = sys.A.Matrices(:,:,i); 
        end
    elseif sys.isA == 1
        sysest.Structure_.A.Parameter.Value = sys.A; 
    end
    
    if sys.isB == 2
        for i = 1:sys.npsiB
           sysest.Structure_.B.Parameter(i).Value = sys.B.Matrices(:,:,i); 
        end
    elseif sys.isB == 1
        sysest.Structure_.B.Parameter.Value = sys.B; 
    end
    
    if sys.isC == 2
        for i = 1:sys.npsiC
           sysest.Structure_.C.Parameter(i).Value = sys.C.Matrices(:,:,i); 
        end
    elseif sys.isC == 1
        sysest.Structure_.C.Parameter.Value = sys.C; 
    end
    
    if sys.isD == 2
        for i = 1:sys.npsiD
           sysest.Structure_.D.Parameter(i).Value = sys.D.Matrices(:,:,i); 
        end
    elseif sys.isD == 1
        sysest.Structure_.D.Parameter.Value = sys.D; 
    end
    
    if sys.isK == 2
        for i = 1:sys.npsiK
           sysest.Structure_.K.Parameter(i).Value = sys.K.Matrices(:,:,i); 
        end
    elseif sys.isK == 1
        sysest.Structure_.K.Parameter.Value = sys.K; 
    end

    if strcmpi(options.InitialState,'estimate')
        sysest.Structure_.X0 = sys.x0;
    end
    
    
	%% Post-processing: create results report.
    
    yh = LPVcore.numerics.lpvssest_predict(data.InputData, rho, data.OutputData, sys);
    
    eh = (data.OutputData-yh);
    N = size(data,1);
    
    sysest.NoiseVariance = eh'*eh/(N-1);
    
	ver = version('-release');
    ver = str2double(ver(1:4));
    
    if ver > 2017
        sysest.Report_ = createReport(sysest.Report_, yh, sys.nt, data, options, ...
            LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
            ctrlMsgUtils.message('Ident:estimation:msgStatusPredictionFocusWithOptions','LPV PEM-SS', strcat('using:',lower(options.SearchMethod))));
    else
        sysest.Report_ = createReport(sysest.Report_, yh, sys.nt, data, options, ...
            LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
            ctrlMsgUtils.message('Ident:estimation:msgStatusValue',['LPV SS using: ', lower(options.SearchMethod)],'prediction'));
    end
    

    if Disp && DispWind.ActiveJavaWindow
        idpack.EstimProgress.displayResults(DispWind, sysest.Report);
        DispWind.STOP = true;
    end
    
    %% Post-Processing: add information from data set to model
    
    if ~isempty(data.InputName);      sysest.InputName      = data.InputName; end
    if ~isempty(data.InputName);      sysest.InputUnit      = data.InputUnit; end
    if ~isempty(data.SchedulingName); sysest.SchedulingName = data.SchedulingName; end
    if ~isempty(data.SchedulingUnit); sysest.SchedulingUnit = data.SchedulingUnit; end
    if ~isempty(data.OutputName);     sysest.OutputName     = data.OutputName; end
    if ~isempty(data.OutputName);     sysest.OutputUnit     = data.OutputUnit; end
    
    sysest.Ts = data.Ts;
    sysest.TimeUnit = data.TimeUnit;
    
end



function stop = localOutFunFmincon(~,optimValues,state, DispWind)
     stop = false;
     
     if strcmpi(state,'iter')
        if DispWind.ActiveJavaWindow
            idDisplayEstimationInfo('Progress', sprintf(' %5d   %7d    %1.4e     %1.4e     %1.4e     %1.4e', ...
                        optimValues.iteration, optimValues.funccount, optimValues.fval, optimValues.constrviolation, optimValues.firstorderopt, optimValues.trustregionradius), DispWind);
        end
        drawnow % Else will the stop function not work
     end
     
     if DispWind.STOP == true
         stop = true;
     end
end 

function stop = localOutFunLsqnonlin(~,optimValues,state, DispWind)
     stop = false;
     
     if strcmpi(state,'iter')
        if DispWind.ActiveJavaWindow
            idDisplayEstimationInfo('Progress', sprintf(' %5d   %7d    %1.4e     %1.4e     %1.4e     %1.4e', ...
                        optimValues.iteration, optimValues.funccount, optimValues.resnorm, optimValues.firstorderopt, optimValues.stepsize), DispWind);
        end
        drawnow % Else will the stop function not work
     end
     
     if DispWind.STOP == true
         stop = true;
     end
end 
