function [sys, x0] = lpvssest(data, init_sys, options)
%LPVSSEST Estimating an LPV-SS model from data    
%
%   sys = lpvssest(data, init_sys)
%      estimates an LPV-SS model from data using the input-scheduling-ouput
%      data DATA.
%
%   DATA is the time-domain estimation data captured in an LPVIDDATA
%      object. Type "help lpviddata" for more information.
%   INIT_SYS is an initial LPV-SS model. This can either be an LPVSS or
%      LPVIDSS object.
%
%   sys = lpvssest(data, init_sys, options)
%   [sys, x0] = lpvssest(__)
%
%   OPTIONS specifies the options of the estimation methodology. Use the
%      "lpvssestOptions" command to configure OPTIONS.
%
%   x0 is the estimated initial state.
%
% See also PMATRIX, LPVSSESTOPTIONS

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    narginchk(2,inf);     % Validate correct number of inputs

    ni = nargin;
    
    if ~isa(data, 'lpviddata')
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpviddata object','lpvssest');
    end
    
    if ~isdt(init_sys)
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','discrete-time LPV SS representation','lpvssest');
    end
    
    if init_sys.Ts ~= 1
        assert(init_sys.Ts == data.Ts, ...
            'Sampling time Ts of data and template system must match');
    end
    
    if (isempty(init_sys.A) && isempty(init_sys.B) && isempty(init_sys.K)) || isempty(init_sys.C)
        LPVcore.error('LPVcore:general:InvalidInput','The initial LPV-SS model has either no state dynamics or the state is not observed in the output (C is empty).','lpvidss/lpvssest_');
    end
    
    if ~isa(init_sys, 'lpvss') && ~isa(init_sys, 'lpvidss')
        LPVcore.error('LPVcore:general:InvalidInputArgument','second','lpvss or lpvidss object','lpvssest');
    end
    
    [~,ny,np,nu] = size(data);

    if ~isequal(size(init_sys),[ny nu]) || np ~= init_sys.SchedulingDimension
        LPVcore.error('LPVcore:general:InvalidInput','Data and model object should have equivalent input, scheduling, and output dimension','lpvssest');
    end
    
    if ny == 0
        LPVcore.error('LPVcore:general:InvalidInput','There is no output defined and, therefore, minimizing w.r.t. the output is not possible.','lpvssest');
    end
    
    if ni > 2
        if ~isa(options, 'lpvssestOptions')
            LPVcore.error('LPVcore:general:InvalidInputArgument','third','lpvssestOptions object','lpvssest');
        end
    else
        options = lpvssestOptions; 
    end
    
    if isa(init_sys, 'lpvss'); init_sys = lpvidss(init_sys); end
    
    if strcmp(options.SearchMethod,'em') && strcmp(typeNoiseModel(init_sys),'innovation') && ~isempty(init_sys.K)
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with general noise model','lpvssest');
    elseif ~strcmp(options.SearchMethod,'em') && strcmp(typeNoiseModel(init_sys),'general')
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with innovation noise model','lpvssest');
    end
    
    if strcmp(typeNoiseModel(init_sys),'general') && (ispmatrix(init_sys.G) || ispmatrix(init_sys.H))
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','lpvidss object with parameter independent general noise model','lpvssest');
    end

    [sys, x0] = lpvssest_(init_sys,data,options);
    
end

