function [sys, theta] = lpvidss2struct(sysinit,options)
% Create sys structure for the nonlinear optimization methods.
%
% [sys, theta] = lpvidss2struct(sysinit,options)
%
% sysinit is an lpvidss object.
% options is an lpvssestOptions object.
%
% sys is a structure with all necessary information.
% theta is the vector of parameters.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    theta = getpvec(sysinit);

    estInit = strcmpi(options.InitialState,'estimate');
    nx = order(sysinit);

    if estInit
        if isempty(sysinit.X0)
            theta = cat(1,theta,zeros(nx,1));
        else
            theta = cat(1,theta,sysinit.X0);
        end
    end

    nt = numel(theta);      % Amount of parameters to be estimated

    sys = struct('A', sysinit.A, 'B', sysinit.B, 'C', sysinit.C, 'D', sysinit.D, ...
        'nt', nt, 'nx', nx, 'nu', sysinit.Nu, 'ny', sysinit.Ny); % Compile all necessary system info
    
    if strcmpi(options.SearchMethod,'em')
        
        sys.G = sysinit.G;
        sys.H = sysinit.H;
        
        if ~isempty(sysinit.Structure.X0CovT.R)
            sys.P1s = idpack.cholcov(getValue(tmp),'upper');
        else
            sys.P1s = eye(nx);
        end
        
        sys.NoiseVariance = sysinit.NoiseVariance;
        
    else
    
        sys.K = sysinit.K;
        sys.npsiA = numel(sys.A.bfuncs);
        sys.npsiB = numel(sys.B.bfuncs);
        sys.npsiC = numel(sys.C.bfuncs);
        sys.npsiD = numel(sys.D.bfuncs);
        sys.npsiK = numel(sys.K.bfuncs);
    
    end
    
    
    if estInit; sys.x0 = theta(end-nx+1:end);
    else;       sys.x0 = []; end
    sys.isX0= estInit;
    
end