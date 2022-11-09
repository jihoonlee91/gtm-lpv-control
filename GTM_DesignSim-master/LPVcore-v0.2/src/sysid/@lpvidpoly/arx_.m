function sysest = arx_(sysest, data, options, optionsRegul)
%LPVARX Extimate an LPV-ARX model from data    
%
%   sys = arx_(sys, data)
%      estimates an LPV-ARX model from data using the input-ouput data
%      DATA.
%
%   SYS is an LPVIDPOLY object. This object contains the free to be
%      estimated parameters. Note that the ARX_ will overwrite the current
%      coefficients in the pmatrix objects in sys.A and sys.B. The
%      basisfunctions and model structure are kept.
%   DATA is the time-domain estimation data captured in an LPVIDDATA
%      object. Type "help lpviddata" for more information.
%
%   sys = arx_(data, orders, ..., options)
%   sys = arx_(data, orders, ..., options,regulOptions)
%      specifies the options of the estimation methodology. Use the
%      "lpvarxOptions" command to configure OPTIONS and
%      "lpvarxRegulOptions" command to specify REGULOPTIONS.
%
%
% See also LPVARX, LPVARXOPTIONS, LPVARXREGUL, LPVARXREGULOPTIONS

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    %% Input validation

    narginchk(2,4);       % Validate correct number of inputs
    
    if ~iseye(sysest.C) || ~iseye(sysest.D) || ~iseye(sysest.F)
        LPVcore.error('LPVcore:general:InvalidInput','The model is not in an LPV-ARX form','lpvidpoly/arx_');
    end
    
    if sysest.Ts == 0
        LPVcore.error('LPVcore:general:InvalidInputArgument','first','discrete-time LPV IO representation','lpvidpoly/arx_');
    end
    
    if ~isa(data, 'lpviddata')
        LPVcore.error('LPVcore:general:InvalidInputArgument','second','lpviddata object with an input signal','lpvidpoly/arx_');
    end
    
    if nargin > 2
        if ~isa(options, 'lpvpolyestOptions')
            LPVcore.error('LPVcore:general:InvalidInputArgument','third','lpvarxOptions object','lpvidpoly/arx_');
        end
    else
        options     = lpvarxOptions;
    end
    
    if nargin > 3
        if ~isempty(optionsRegul) && ~isa(optionsRegul, 'LPVcore.idoptions.lpvarxRegul')
            LPVcore.error('LPVcore:general:InvalidInputArgument','fourth','lpvarxRegulOptions object','lpvidpoly/arx_');
        end
    else
        optionsRegul = [];  % if optionsRegul not assigned, do not optimize
    end
    
    [~,ny,np,nu] = size(data);

    if ~isequal(size(sysest),[ny nu]) || np ~= sysest.Np
        LPVcore.error('LPVcore:general:InvalidInput','Data and model object should have equivalent input, scheduling, and output dimension','lpvidpoly/arx_');
    end
    
    %% GUI
    h = progressgui(sysest, data, options);
    progressmsg(h, 'Estimating model parameters...');
    progressmsg(h, 'Iterative update does not apply to this estimation.');
    
    %% Pre-processing: Apply InputDelay and remove trend
    inputDelay = sysest.InputDelay;
    for i=1:sysest.Nu
        data.u(:, i) = [ zeros(inputDelay(i), 1); ...
            data.u(1:end-inputDelay(i), i) ];
    end
    data = rmOffset(data, options.OutputOffset,...
                       options.SchedulingOffset, options.InputOffset);
                        
	%% Estimation
    W = options.OutputWeight;
    if isempty(W);  W = eye(ny);
    else
        if size(W,1) ~= ny
            LPVcore.error('LPVcore:general:InvalidOption','OutputWeight','lpvarxOptions',sprintf('%d-by-%d positive definite matrix',ny,ny), 'lpvidpoly/arx_');
        end
        W = idpack.cholcov(W,'lower'); 
    end
    
    [JM, yM] = createDataMatrix(sysest, data);  % Get data matrices.  e = J*theta
    
    yW = W*yM; yW = yW(:);    % Weight the data matrices and vectorize it
    parmFree = isfree(sysest);
    JW = kron(JM, W);
    JW = JW(:, parmFree);

    R1 = triu(qr([JW, yW],0));  % Compress data by QR factorization
    
    npar = numel(parmFree);
    nparfree = sum(parmFree);
    na = sysest.Na;
    nb = sysest.Nb;
    na = na(1);  nb = nb(1);
    
    n = max(na,nb);
    NparEff = nparfree + n;
    Ncapeff = size(data,1)-NparEff;
    
    % Validate that there are sufficient samples
    if size(R1,1) < nparfree + 1
        ctrlMsgUtils.error('Ident:estimation:tooFewSamples')
    end
    
    % Get parameter bounds
    [Min, Max] = getpMinMax(sysest);
    Min = Min(parmFree); Max = Max(parmFree);
    Free = false(npar, 1);
    Free(parmFree) = true;
    
    if all(isinf(Min)) && all(isinf(Max))
        ParBoundsExist = false;
        Min = []; Max = [];
    else
        ParBoundsExist = true;
    end

    % Regularization parameters
    if ~isempty(optionsRegul)
        [Regul.Lambda, Regul.R, Regul.OptionsUsed] = ...
            regul_LS(sysest, R1(1:nparfree,1:nparfree), R1(1:nparfree,nparfree+1), ...
            eye(ny), size(data,1), optionsRegul);
        
        Regul.OptionsUsed.InputOffset      = options.InputOffset;
        Regul.OptionsUsed.SchedulingOffset = options.SchedulingOffset;
        Regul.OptionsUsed.OutputOffset     = options.SchedulingOffset;
    else
        if ~isempty(options.Regularization.Lambda) && options.Regularization.Lambda ~= 0
            Regul = options.Regularization;
        else
            Regul.Lambda = 0;
            Regul.R      = [];
            Regul.OptionsUsed = [];
        end
    end
        
    %% Estimation
    % Copy of idpack.polydata.arx_time
    [parvecTry, R1r] = idpack.lsqbound(zeros(npar,1), R1(1:nparfree,1:nparfree), ...
        R1(1:nparfree,nparfree+1), Free, options.LinearSolver, ...
        ParBoundsExist, Min, Max, Regul);
    
    if any(~isfinite(parvecTry))
        parvec = idpack.lsqbound(zeros(npar,1), R1(1:nparfree,1:nparfree), ...
            R1(1:nparfree,nparfree+1), Free, @idpack.pinvcov, ...
            ParBoundsExist, Min, Max, Regul);
    else
        parvec = parvecTry;
    end
    
    % Unvectorize it again
    parM = reshape(parvec,[ny, npar/ny]);

    % Compute prediction error
	yp = parM*JM';
    e = yM - yp;
    
    NoiseVariance = e*e'/Ncapeff;     % Lambda0  (9.47)  <- semi-positive definite
    
    if (options.EstimateCovariance && Ncapeff>0)
        % Compute covariance
        SigSqrd = W*idpack.cholcov(NoiseVariance,'lower');
        
        % psi * Sigma *psi'  (9.47)
        JSmat = kron(JM, SigSqrd);
        R1f = triu(qr(JSmat(:, parmFree)));
        [nRrf,nRcf] = size(R1f);
        R1f = R1f(1:min(nRrf, nRcf), :);
        
        if Regul.Lambda > 0
            R1 = R1r;  % Get regularized, weighted data matrix
        end
        
        R1 = R1(1:nparfree,1:nparfree);
        R3 = triu(qr(R1f'\R1'));

        if hasInfNaN(R3)
            R3 = triu(qr(idpack.mldividecov(R1f', R1', 1e-6)));
        end
        cov = idpack.FactoredCovariance(R3*R1, [], Free);  % Note that this is cov = inv(R1'*R3'*R3*R1)
        sysest.Covariance = cov;
        
        % Getting the covariance: getValue(sysest.Covariance)
    end
    
    
    %% Set the parameters in sysest
    ct = 1;
    for i=1:na
        if ~isempty(sysest.A.MatNoMonic{i})
            npsi = numel(sysest.A.MatNoMonic{i}.bfuncs);
            sysest.A.Mat{i+1}.matrices = reshape(parM(:,ct:ct+ny*npsi-1), ...
                size(sysest.A.MatNoMonic{i}.matrices) );
            ct = ct+ny*npsi;
        end
    end
    
    for i=1:nb+1
        if ~isempty(sysest.B.Mat{i})
            npsi = numel(sysest.B.Mat{i}.bfuncs);
            sysest.B.Mat{i}.matrices = reshape(parM(:,ct:ct+nu*npsi-1), ...
                size(sysest.B.Mat{i}.matrices) );
            ct = ct+nu*npsi;
        end
    end
    
	[~,p] = chol(NoiseVariance);
    if p ~= 0   
        % Noise variance is numerically not positive definite. We will make
        % the matrix positive definite
        NV = idpack.cholcov(NoiseVariance,'lower');
       	sysest.NoiseVariance = NV'*NV;
        msg = LPVcore.message('LPVcore:lpvidpoly:MakeNoiseVarPD');
        warning(msg.identifier, msg.message);
    else
        sysest.NoiseVariance = NoiseVariance;
    end
    
    %% Post-processing: create results report.
    
    h = terminate(h, sysest, SysIdTermConditions.NonIterative);
    
    sysest.Report = h.Report;
    
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

function [J, e] = createDataMatrix(sys, data)
    % CREATEDATAMATRIX Create data matrix for the LSQ solver for all
    % parameters (including non-free).
    %
    %   Syntax:
    %       [J, e]  = createDataMatrix(sys, data)
    %
    %   Inputs:
    %       sys: lpvidpoly objects
    %       data: lpviddata object
    %
    %   Outputs:
    %       J: ny-by-N data matrix.
    %       e: ny-by-N data matrix.
    %

    npar = nparams(sys);
    
    [N, ny, ~, nu]  = size(data);
    na = sys.Na;
    nb = sys.Nb;
    na = na(1);  nb = nb(1);
    
    n = max(na,nb);
    
    J = zeros(npar/ny,N-n);
    e = data.y(1+n:end,:)';

    % Add the A poly to J
    cnt = 1;
    
    rho = createRho(sys.SchedulingTimeMap, data.p);
    for i=1:na
        if ~isempty(sys.A.MatNoMonic{i})
            % Including basis functions
            AFeval = feval(sys.A.MatNoMonic{i}, rho, 'basis');
            npt = size(AFeval,2);
            tmpSrho = repmat(1:npt,[ny 1]); tmpSrho = tmpSrho(:)';
            J(cnt:cnt+npt*ny-1,:) = (-AFeval(1+n:end,tmpSrho).*repmat(data.y(1+n-i:end-i,:),[1 npt]) )';
            cnt = cnt + npt*ny;
        end
    end
    
    for i=1:nb+1
        if ~isempty(sys.B.Mat{i})
            % Including basis functions
            BFeval = feval(sys.B.Mat{i}, rho, 'basis');
            npt = size(BFeval,2);
            tmpSrho = repmat(1:npt,[nu 1]); tmpSrho = tmpSrho(:)';
            J(cnt:cnt+npt*nu-1,:) = ( BFeval(1+n:end,tmpSrho).*repmat(data.u(2+n-i:end-i+1,:),[1 npt]) )';
            cnt = cnt + npt*nu;
        end
    end
    
    J = J';
end
