function sys = lpvssest_em(sysinit, data, options, DispWind)
%LPVSSEST_EM The Expectation-Maximization
%
%   Does an iteration between an Kalman filter with Rauch-Tung-Striebel
%   smoother and the Expectation-Maximization [1, Sec. 7.3].
%
%   [x(t+1) = [A0,...,Anp,B0,...,Bnp  [kron([1;p(t)],x(t))   +  [w(t)
%    y(t) ]    C0,...,Cnp,D0,...,Dnp]  kron([1;p(t)],u(t))]      v(t)]
%
%   where
%   [w(t)   ~  N(0, Pi)
%    v(t)]
%
%   [sys,x1,Pi,P1,xhtN] = emlpv(x, u, p, y, sysint, x1, Pi, P1, maxIter)
%   
%   x       The true state signal. Size nx x N, where nx state dimension
%           and N the signal length. Only used to compare internal
%           convergence. Should be removed in newer versions. 
%   u       The input signal. Size nu x N, where nu input dimension and N
%           the signal length.
%   p       The scheduling signal. Size np x N, where np scheduling
%           dimension and N the signal length. 
%   y       The output signal. Size ny x N, where ny output dimension and N
%           the signal length.
%   sysint  Initial LPV-SS system.
%   x1      Initial state of the LPV-SS system.
%   Pi      Initial covariance of the noise on the state and output.
%   P1      Initial covariance on the initial state.
%   maxIter Maximum amount of iterations.
%
%   sys     Final system.
%   x1      Initial state after iterations.
%   Pi      Final covariance of the noise on the state and output.
%   P1      Final covariance on the initial state.
%   xhtN    Final filtered state signal.
%
% [1] Cox, P. B. (2018). Towards Efficient Identification of Linear
% Parameter-Varying State-Space Models. Eindhoven University of Technology.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

nx = sysinit.StateSize;
ny = sysinit.IOSize(1);
nu = sysinit.IOSize(2);
N = size(data,1);
DispWind.STOP = false;

estInit = strcmpi(options.InitialState,'estimate');
Disp = ~strcmpi(options.Display,'off');

sysinit.G = eye(nx);
sysinit.H = eye(ny);


sysinit = combineBasisFunc(sysinit);

    % Collect all essential information
sys = LPVcore.numerics.lpvidss2struct(sysinit,options);


% Pre-compute the extended scheduling signal rho(t)
if ~isempty(sysinit.Structure_.SchedulingTimeMap)
	rho = createRho(sysinit.Structure_.SchedulingTimeMap, data.SchedulingData); %, pinit, pterm);
else
    rho = [];
end

% precompute the matrices over the scheduling trajectory
if     sys.isA == 2; psiA = feval(sys.A,rho,'basis');
elseif sys.isA == 1; psiA = ones(N,1); end

if     sys.isB == 2; psiB = feval(sys.B,rho,'basis');
elseif sys.isB == 1; psiB = ones(N,1); end


% W = options.OutputWeight;
% if isempty(W)
%     W = eye(ny);
% else
%     LPVcore.error('LPVcore:general:InvalidOption','OutputWeight','lpvssestOptions','empty matrix for enhanced gauss newton method', 'lpvidss/lpvssest_');
% end



iter = 0;       % iteration number
relTolQ = options.SearchOptions.RelTolFun;     % relative tolerance of the marginal log likelihood to abort iterations
absTolQ = options.SearchOptions.AbsTolFun;     % absolute tolerance of the marginal log likelihood to abort iterations
relTolP = options.SearchOptions.AbsTolParm;    % relative tolerance of the parameter fluctuations to abort iterations
absTolP = options.SearchOptions.AbsTolParm;    % absolute tolerance of the parameter fluctuations to abort iterations
Qre_min = options.SearchOptions.RelTolFunInit; % the minimal relative marginal log likelihood value change to abort iterations. This will be the percentage which it should drop from the initial
maxIter = options.SearchOptions.maxIter;

Qn = inf;       % inital marginal log likelihood on step iter
Parmo = inf;    % inital absolute parameter variation on step inter-1
RelTolQ_ = inf;
AbsTolQ_ = inf;
RelTolP_ = inf;
AbsTolP_ = inf;
Q_min = -inf;


if Disp && DispWind.ActiveJavaWindow
    idDisplayEstimationInfo('Progress',{' ',LPVcore.messageStr('LPVcore:lpvidss:tabHearderEM1'),LPVcore.messageStr('LPVcore:lpvidss:tabHearderEM2')},DispWind);
end


      
    % The iteration will be stoped if the maximum amound of iterations
    % (maxIter) is reached or when the following 5 conditions are
    % satisfied:
    % 1. The marginal log likelihood has a relative change smaller than
    %    relTolQ.
    % 2. The marginal log likelihood has an absolute change smaller than
    %    absTolQ.
    % 3. The abs sum of the system parameters has a relative change w.r.t.
    %    the previous iteration smaller than relTolP. 
    % 4. The abs sum of the system parameters has an absolute change w.r.t.
    %    the previous iteration smaller than absTolP. 
    % 5. The marginal log likelihood is smaller the Q_min. This value will
    %    be determined on the first iteration and will be Qre_min %
    %    smaller.
    
while (iter < maxIter) && ( RelTolQ_ > relTolQ || AbsTolQ_ > absTolQ || ...
                            RelTolP_ > relTolP || AbsTolP_ > absTolP || ...
                            Qn > Q_min) && DispWind.STOP == false

%% E-step, estimate the state and covariance matrices. 
                        
                        
	[xhtN, PtN, MN, P1_s] = LPVcore.numerics.lpvssest_em_filters(sys, data.OutputData, rho, data.InputData);
        
        
%% E-step, compute the data matrices.

%         % ---- Begin: Slow construction Phi, Psi, Sigma
%         % Be aware that the fast algorithm changes dimension on PtN. So
%         % use PtN(:,t,:) in stead of PtN(:,:,t)
% 	Phi1 = zeros(nx+ny,nx+ny);
%     Psi1 = zeros(nx+ny,sys.npsiA*nx+sys.npsiB*nu);
%     Sigma1 = zeros(sys.npsiA*nx+sys.npsiB*nu,sys.npsiA*nx+sys.npsiB*nu);
%     
% 
%     for t = 1:N
% 
%         Phi1 = Phi1+ [ xhtN(:,t+1)*xhtN(:,t+1)'+PtN(:,:,t+1)  , xhtN(:,t+1)*data.OutputData(t,:)
%                      data.OutputData(t,:)'*xhtN(:,t+1)'       , data.OutputData(t,:)'*data.OutputData(t,:)];
% 
%         Psi1 = Psi1+ [  kron(psiA(t,:), xhtN(:,t+1)*xhtN(:,t)' + MN(:,:,t+1)), kron(psiB(t,:), xhtN(:,t+1)*data.InputData(t,:))
%                       kron(psiA(t,:), data.OutputData(t,:)'*xhtN(:,t)')                    , kron(psiB(t,:), data.OutputData(t,:)'*data.InputData(t,:)) ];
% 
% 
%         Sigma1 = Sigma1+  [  kron(psiA(t,:)'*psiA(t,:), xhtN(:,t)*xhtN(:,t)' + PtN(:,:,t)), kron(psiA(t,:)'*psiB(t,:), xhtN(:,t)*data.InputData(t,:))
%                            kron(psiB(t,:)'*psiA(t,:), data.InputData(t,:)'*xhtN(:,t)')  , kron(psiB(t,:)'*psiB(t,:), data.InputData(t,:)'*data.InputData(t,:)) ];
% 
% 
%     end
%         % ---- End: Slow construction Phi, Psi, Sigma

        
        % ---- Begin: Fast construction of Phi, Psi, Sigma
    Phi = zeros(nx+ny,nx+ny);

    Phi(1:nx,1:nx) = xhtN(:,2:end)*xhtN(:,2:end)'+sum(PtN(:,:,2:end),3);
    
    Phi(1:nx,nx+1:nx+ny) = xhtN(:,2:end)*data.OutputData;
    
    Phi(nx+1:nx+ny,1:nx) = Phi(1:nx,nx+1:nx+ny)';
    
    Phi(nx+1:nx+ny,nx+1:nx+ny) = data.OutputData'*data.OutputData;

    
    Psi = zeros(nx+ny,sys.npsiA*nx+sys.npsiB*nu);
    
    MN = permute(MN,[1 3 2]);      % Change dimensions for faster computation
    
    for i = 1:sys.npsiA
        for j = 1:nx
            for k = 1:nx
                Psi(j, (i-1)*nx+k) = ( xhtN(j,2:end).*xhtN(k,1:end-1) + MN(j,2:end,k) ) * psiA(:,i);
            end
        end
    end
    
    for i = 1:sys.npsiB
        for j = 1:nx
            for k = 1:nu
                Psi(j, sys.npsiA*nx+(i-1)*nu+k) = ( xhtN(j,2:end).*data.InputData(:,k)' ) * psiB(:,i);
            end
        end
    end
    
	for i = 1:sys.npsiA
        for j = 1:ny
            for k = 1:nx
                Psi(nx+j, (i-1)*nx+k) = ( data.OutputData(:,j)'.*xhtN(k,1:end-1) ) * psiA(:,i);
            end
        end
	end
    
	for i = 1:sys.npsiB
        for j = 1:ny
            for k = 1:nu
                Psi(nx+j, sys.npsiA*nx+(i-1)*nu+k) = ( data.OutputData(:,j).*data.InputData(:,k) )'* psiB(:,i);
            end
        end
	end
    
    
	Sigma = zeros(sys.npsiA*nx+sys.npsiB*nu,sys.npsiA*nx+sys.npsiB*nu);
    
    PtN = permute(PtN,[1 3 2]);      % Change dimensions for faster computation
        
    for i = 1:sys.npsiA
        for j = 1:sys.npsiA
            for k = 1:nx
                for l = 1:nx
                    Sigma((i-1)*nx+k, (j-1)*nx+l) = ( xhtN(k,1:end-1).*xhtN(l,1:end-1) +PtN(k,1:end-1,l) ) * (psiA(:,i).*psiA(:,j));
                end
            end
        end
    end
    
	for i = 1:sys.npsiA
        for j = 1:sys.npsiB
            for k = 1:nx
                for l = 1:nu
                    Sigma((i-1)*nx+k, sys.npsiA*nx+(j-1)*nu+l) = ( xhtN(k,1:end-1).*data.InputData(:,l)' )*( psiA(:,i).*psiB(:,j) );
                end
            end
        end
	end
    
    
    Sigma(sys.npsiA*nx+1:end,1:sys.npsiA*nx) = Sigma(1:sys.npsiA*nx,sys.npsiA*nx+1:end)';
    
	for i = 1:sys.npsiB
        for j = 1:sys.npsiB
            for k = 1:nu
                for l = 1:nu
                    Sigma(sys.npsiA*nx+(i-1)*nu+k, sys.npsiA*nx+ (j-1)*nu+l) = ( data.InputData(:,k).*data.InputData(:,l) )' * ( psiB(:,i).*psiB(:,j) );
                end
            end
        end
	end
   
        % ---- End: Fast construction of Phi, Psi, Sigma

    Phi = Phi/N;
    Psi = Psi/N;
    Sigma = Sigma/N;


%% M - step

% %   Compute the estimate by Matlab inverse
%     Gammah = Psi / Sigma;

    Rfac = triu(qr([Sigma Psi']));
    R1fac = Rfac(:,1:sys.npsiA*nx+sys.npsiB*nu);
    R2fac = Rfac(:,sys.npsiA*nx+sys.npsiB*nu+1:end);
    
    Gammah = idpack.mldividecov(R1fac,R2fac,1e-12)';  %     Gammah = R2fac' / R1fac';
    
    
    [Rch,sucC] = chol([Sigma, Psi'; Psi, Phi]);
    
    if(sucC ~= 0)
%       Abort if chol cannot be computed anymore. In future versions we
%       could include in this case an eigenvalue decomposition. In my
%       experience this case only happens when the solution diverges

        idDisplayEstimationInfo('Progress','Cannot update covariance of the noise. Most likely did the system not converge and resulted in an unstable system. Try again with different initial guess.',DispWind);
        break;
    end
    
    sys.x0 = xhtN(:,1);     % Update initial state
    sys.P1s = P1_s;         % Update initial covariance of state

    sys.NoiseVariance = (Rch(end-(nx+ny)+1:end,end-(nx+ny)+1:end))' * ...
          Rch(end-(nx+ny)+1:end,end-(nx+ny)+1:end); % Update NoiseVariance
      
	sys = gamma2sys(Gammah,sys);
    
        % Update Qo and iter
    iter = iter+1;
    Qo = Qn;
    P1 = P1_s'*P1_s;
    x1 = sys.x0;
    Pi = sys.NoiseVariance;
    Qn = log(det(P1)) + N*log(det(sys.NoiseVariance)) + trace( P1 \ (xhtN(:,1)*xhtN(:,1)'+squeeze(PtN(:,1,:)) -xhtN(:,1)*x1'-x1*xhtN(:,1)' +x1*x1' ) ) ...
            + trace(Pi \ ( Phi-Psi*Gammah'-Gammah*Psi'+Gammah*Sigma*Gammah' ));

    AbsTolQ_ = abs(Qn - Qo);
    RelTolQ_ = AbsTolQ_ / min(abs(Qn), abs(Qo));
    AbsTolP_ = sum(abs(Parmo - Gammah(:)))/numel(Parmo);
    RelTolP_ = sum(abs(Parmo - Gammah(:))) / min(sum(abs(Parmo)), min(sum(Gammah(:))));

    Parmo = Gammah(:);
    
    if iter == 1
        Q_min = (1-sign(Qn)*Qre_min)*Qn; % sign is needed such that negative number becomes more negative
    end

    if Disp && DispWind.ActiveJavaWindow
        if iter == 1
            idDisplayEstimationInfo('Progress',sprintf(' %5d      %1.4e',iter, Qn),DispWind);
        else
            idDisplayEstimationInfo('Progress',sprintf(' %5d      %1.4e      %1.4e      %1.4e      %1.4e      %1.4e      %1.4e',iter, Qn, AbsTolQ_, RelTolQ_,AbsTolP_,RelTolP_),DispWind);
        end
        drawnow; % update GUI handles to obtain correct DispWind.STOP
    end
    
end


    %% Post-processing: Save data in the lpvidss object
    
    if sys.isA == 2
        for i = 1:sys.npsiA
           sysinit.Structure_.A.Parameter(i).Value = sys.A.Matrices(:,:,i); 
        end
    elseif sys.isA == 1
        sysinit.Structure_.A.Parameter.Value = sys.A; 
    end
    
    if sys.isB == 2
        for i = 1:sys.npsiB
           sysinit.Structure_.B.Parameter(i).Value = sys.B.Matrices(:,:,i); 
        end
    elseif sys.isB == 1
        sysinit.Structure_.B.Parameter.Value = sys.B; 
    end
    
    if sys.isC == 2
        for i = 1:sys.npsiC
           sysinit.Structure_.C.Parameter(i).Value = sys.C.Matrices(:,:,i); 
        end
    elseif sys.isC == 1
        sysinit.Structure_.C.Parameter.Value = sys.C; 
    end
    
    if sys.isD == 2
        for i = 1:sys.npsiD
           sysinit.Structure_.D.Parameter(i).Value = sys.D.Matrices(:,:,i); 
        end
    elseif sys.isD == 1
        sysinit.Structure_.D.Parameter.Value = sys.D; 
    end

    if estInit
        sysinit.Structure_.X0 = sys.x0;
        sysinit.Structure_.X0CovT = idpack.FactoredCovariance(idpack.mldividecov(sys.P1s',eye(nx)),[],[]);  % Note that this is cov = inv(R1'*R3'*R3*R1)
    end


	%% Post-processing: create results report.
    
    yh = LPVcore.numerics.lpvssest_em_predict(sys,data.OutputData, rho, data.InputData );
    
    sysinit.NoiseVariance = sys.NoiseVariance;
    
    	ver = version('-release');
    ver = str2double(ver(1:4));
    
    if ver > 2017
        sysinit.Report_ = createReport(sysinit.Report_, yh, sys.nt, data, options, ...
            LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
            ctrlMsgUtils.message('Ident:estimation:msgStatusPredictionFocusWithOptions','LPV PEM-SS', 'using: Expectation-Maximization Method'));
    else
        sysinit.Report_ = createReport(sysinit.Report_, yh, sys.nt, data, options, ...
            LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
            ctrlMsgUtils.message('Ident:estimation:msgStatusValue','LPV SS using: Expectation-Maximization Method','prediction'));
    end
     
    if Disp && DispWind.ActiveJavaWindow
        idpack.EstimProgress.displayResults(DispWind, sysinit.Report);
        DispWind.STOP = true;
    end
    
    %% Post-Processing: add information from data set to model
    
    if ~isempty(data.InputName);      sysinit.InputName      = data.InputName; end
    if ~isempty(data.InputName);      sysinit.InputUnit      = data.InputUnit; end
    if ~isempty(data.SchedulingName); sysinit.SchedulingName = data.SchedulingName; end
    if ~isempty(data.SchedulingUnit); sysinit.SchedulingUnit = data.SchedulingUnit; end
    if ~isempty(data.OutputName);     sysinit.OutputName     = data.OutputName; end
    if ~isempty(data.OutputName);     sysinit.OutputUnit     = data.OutputUnit; end
    
    sysinit.Ts = data.Ts;
    sysinit.TimeUnit = data.TimeUnit;
    
    
    sys = sysinit;
    
    
end


function sys = gamma2sys(Gamma,sys)

    nx = sys.nx;
    nu = sys.nu;
    ny = sys.ny;
    count = 1;

    if sys.isA == 2
        sys.A.Matrices = reshape(Gamma(1:nx,1:sys.npsiA*nx),nx,nx, sys.npsiA);
        count = count+sys.npsiA*nx;
    elseif sys.isA == 1
        sys.A = Gamma(1:nx,1:nx);
        count = count+nx;
    end

    if sys.isB == 2
        sys.B.Matrices = reshape(Gamma(1:nx,count:count+sys.npsiB*nu-1),nx,nu, sys.npsiB);
    elseif sys.isB == 1
        sys.B = Gamma(1:nx,count:count+nu-1);
    end


    if sys.isC == 2
        sys.C.Matrices = reshape(Gamma(nx+1:end,1:sys.npsiA*nx),ny,nx, sys.npsiC);
        count = sys.npsiA*nx;
    else
        sys.C = Gamma(nx+1:end,1:nx);
        count = nx;
    end

    if sys.isD == 2
        sys.D.Matrices = reshape(Gamma(nx+1:end,count:count+sys.npsiD*nu-1),ny,nu, sys.npsiD);
    elseif sys.isD == 1
        sys.D = Gamma(nx+1:end,count:count+nu-1);

    end

end


function sys = combineBasisFunc(sys)

    A = sys.A;
    if isa(A,'pmatrix'); typeA = getTypeBasis_(A);
    elseif ~isempty(A);  typeA = {0,[]};
    else;                typeA = {};                    
    end
    
	C = sys.C;  % C needs to be nonempty
    if isa(C,'pmatrix'); typeC = getTypeBasis_(C);
    else;                typeC = {0,[]};                 end
    
    if isa(A,'pmatrix') && isa(C,'pmatrix')
        
        [typeBasis,idxA,idxC] = localCombTypeBasis(typeA, typeC);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxA) = A.BasisFunctions;
        basisFun(idxC) = C.BasisFunctions;
        
        MatA = zeros(size(A,1),size(A,2),np);
        MatC = zeros(size(C,1),size(C,2),np);
        
        MatA(:,:,idxA) = A.Matrices;
        MatC(:,:,idxC) = C.Matrices;
        
        sys.A = pmatrix.constructPmatrix(MatA,basisFun,A.SchedulingTimeMap,typeBasis);
        sys.C = pmatrix.constructPmatrix(MatC,basisFun,A.SchedulingTimeMap,typeBasis);
        
    elseif isa(A,'pmatrix') && ~isempty(C)
        
        [typeBasis,idxA,idxC] = localCombTypeBasis(typeA, typeC);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxA) = A.BasisFunctions;
        
        MatA = zeros(size(A,1),size(A,2),np);
        MatC = zeros(size(C,1),size(C,2),np);
        
        MatA(:,:,idxA) = A.Matrices;
        MatC(:,:,idxC) = C;
        
        sys.A = pmatrix.constructPmatrix(MatA,basisFun,A.SchedulingTimeMap,typeBasis);
        sys.C = pmatrix.constructPmatrix(MatC,basisFun,A.SchedulingTimeMap,typeBasis);
        
        
    elseif isa(C,'pmatrix') && ~isempty(A)
        
        [typeBasis,idxA,idxC] = localCombTypeBasis(typeA, typeC);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxC) = C.BasisFunctions;
        
        MatA = zeros(size(A,1),size(A,2),np);
        MatC = zeros(size(C,1),size(C,2),np);
        
        MatA(:,:,idxA) = A;
        MatC(:,:,idxC) = C.Matrices;
        
        sys.A = pmatrix.constructPmatrix(MatA,basisFun,C.SchedulingTimeMap,typeBasis);
        sys.C = pmatrix.constructPmatrix(MatC,basisFun,C.SchedulingTimeMap,typeBasis);
        
    elseif isa(C,'pmatrix')   
        % if C and A constant, nothing to change. Not treated in this if
        % A is empty but not C. Then construct A
        
        sys.A = pmatrix.constructPmatrix(zeros(sys.StateSize,sys.StateSize,C.SchedulingDimension),C.BasisFunctions,C.SchedulingTimeMap,getTypeBasis_(C));
        
    else
        % A is empty but not C. Then construct A
        
        sys.A = zeros(sys.StateSize,sys.StateSize);
    end
    
    
    
	B = sys.B;
    if isa(B,'pmatrix'); typeB = getTypeBasis_(B);
    elseif ~isempty(B);  typeB = {0,[]};
    else;                typeB = {};                    
    end
    
	D = sys.D;  
    if isa(D,'pmatrix'); typeD = getTypeBasis_(D);
    else;                typeD = {0,[]};                 end
    
    if isa(B,'pmatrix') && isa(D,'pmatrix')
        
        [typeBasis,idxB,idxD] = localCombTypeBasis(typeB, typeD);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxB) = B.BasisFunctions;
        basisFun(idxD) = D.BasisFunctions;
        
        MatB = zeros(size(B,1),size(B,2),np);
        MatD = zeros(size(D,1),size(D,2),np);
        
        MatB(:,:,idxB) = B.Matrices;
        MatD(:,:,idxD) = D.Matrices;
        
        sys.B = pmatrix.constructPmatrix(MatB,basisFun,B.SchedulingTimeMap,typeBasis);
        sys.D = pmatrix.constructPmatrix(MatD,basisFun,B.SchedulingTimeMap,typeBasis);
        
    elseif isa(B,'pmatrix') && ~isempty(D)
        
        [typeBasis,idxB,idxD] = localCombTypeBasis(typeB, typeD);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxB) = B.BasisFunctions;
        
        MatB = zeros(size(B,1),size(B,2),np);
        MatD = zeros(size(D,1),size(D,2),np);
        
        MatB(:,:,idxB) = B.Matrices;
        MatD(:,:,idxD) = D;
        
        sys.B = pmatrix.constructPmatrix(MatB,basisFun,B.SchedulingTimeMap,typeBasis);
        sys.D = pmatrix.constructPmatrix(MatD,basisFun,D.SchedulingTimeMap,typeBasis);
        
    elseif isa(B,'pmatrix') %&& isempty(D)
             
        sys.D = pmatrix.constructPmatrix(zeros([sys.IOSize,B.SchedulingDimension]),B.BasisFunctions,B.SchedulingTimeMap,getTypeBasis_(B));
        
    elseif ~isempty(B) && isa(D,'pmatrix')
        
        [typeBasis,idxB,idxD] = localCombTypeBasis(typeB, typeD);
        np = size(typeBasis,1);
        basisFun = cell(1,np);
        basisFun(idxD) = D.BasisFunctions;
        
        MatB = zeros(size(B,1),size(B,2),np);
        MatD = zeros(size(D,1),size(D,2),np);
        
        MatB(:,:,idxB) = B;
        MatD(:,:,idxD) = D.Matrices;
        
        sys.B = pmatrix.constructPmatrix(MatB,basisFun,D.SchedulingTimeMap,typeBasis);
        sys.D = pmatrix.constructPmatrix(MatD,basisFun,D.SchedulingTimeMap,typeBasis);
        
    elseif ~isempty(B)  && isempty(D)  
        % if B and D constant, nothing to change. Not treated in this if
        % if B and D empty, do noting.
        % D is empty but not B. Then construct D
        
        sys.D = zeros(sys.IOSize);
        
    elseif isempty(B) && isa(D,'pmatrix')
        
        sys.B = pmatrix.constructPmatrix(zeros(sys.StataSize,sys.IOSize(2),D.SchedulingDimension),D.BasisFunctions,D.SchedulingTimeMap,getTypeBasis_(D));
        
    elseif isempty(B) && ~isempty(D)
        sys.B = zeros(sys.StataSize,sys.IOSize(2));
    end
    
    
end

function [typeBasis,idx1,idx2] = localCombTypeBasis(type1, type2)
    
    typeBasis = type1;
    idx1 = 1:size(type1,1);
    idx2 = zeros(1,size(type2,1));
    
    cnt = size(type1,1)+1;
    
    numType1 = num2cell(type1,2);
    
    for i = 1:size(type2,1)
        
        idx = find(cellfun(@(x) isequal(x,type2(i,:)), numType1),1);
        
        if isempty(idx) % does not exist
            typeBasis = cat(1,typeBasis,type2(i,:));
            idx2(i) = cnt;
            cnt  = cnt+1;
        else
            idx2(i) = idx;
        end
        
    end

end

