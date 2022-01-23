function sys = lpvssest_egn(sysinit, data, options, DispWind)
%LPVSSEST_EGN Enhanced Gaus-Newton Data-Driven Local Coordinates LPV SS
%estimator
%
%   Detailed explanation goes here
% 
% See [1, Section 7.2].
%
% [1] Cox, P. B. (2018). Towards Efficient Identification of Linear
% Parameter-Varying State-Space Models. Eindhoven University of Technology.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

nx = sysinit.Nx;
ny = sysinit.Ny;
nu = sysinit.Nu;
DispWind.STOP = false;

estInit = strcmpi(options.InitialState,'estimate');
Disp = ~strcmpi(options.Display,'off');

% Collect all essential information
[sys, theta] = LPVcore.numerics.lpvidss2struct(sysinit,options);

nt = numel(theta);

% Pre-compute the extended scheduling signal rho(t)
rho = createRho(sysinit.SchedulingTimeMap, data.SchedulingData);

W = options.OutputWeight;
if isempty(W);  W = eye(ny);
else
    LPVcore.error('LPVcore:general:InvalidOption','OutputWeight','lpvssestOptions','empty matrix for enhanced gauss newton method', 'lpvidss/lpvssest_');
end

% Initialize the optimization parameters
beta      = options.SearchOptions.beta;
gamma     = options.SearchOptions.gamma;
lambdaMIN = options.SearchOptions.lambdaMIN;
alphaMIN  = options.SearchOptions.alphaMIN;
nu_       = options.SearchOptions.nu;
epsilon   = options.SearchOptions.epsilon;
lambda    = lambdaMIN;
iter      = 0;

trmntncond = inf;
absChange = inf;
relChange = inf;

sysOld = sys;

if Disp && DispWind.ActiveJavaWindow
    idDisplayEstimationInfo('Progress',{' ',LPVcore.messageStr('LPVcore:lpvidss:tabHearderEGN1'),LPVcore.messageStr('LPVcore:lpvidss:tabHearderEGN2')},DispWind);
end

while ( trmntncond > epsilon ) && (absChange > options.SearchOptions.AbsTol)  ...
        && (relChange > options.SearchOptions.RelTol) && (iter < options.SearchOptions.maxIter) && (DispWind.STOP == false)
    
    %% Compute descent direction
    
        % ------------------------------------
        % 1. Compute the Q and P space
    
    Q = zeros(nt,nx^2);
    eyenx = eye(nx);
    
    for i = 1:numel(sys.A.bfuncs)
        Q(nx^2*(i-1)+1 : nx^2*i,  : ) = ...
             kron(sys.A.matrices(:,:,i)', eyenx) - ...
             kron(eyenx, sys.A.matrices(:,:,i));
    end

    cnt = numel(sys.A.bfuncs) * nx^2;
    
    for i = 1:numel(sys.B.bfuncs)
        Q( cnt + nu*nx*(i-1) + 1 : cnt+ nu*nx*i, : ) = ...
             kron(sys.B.matrices(:,:,i)', eyenx);
    end
    
    cnt = cnt + numel(sys.B.bfuncs) * nu*nx;
    
    for i = 1:numel(sys.C.bfuncs)
        Q( cnt + ny*nx*(i-1) +1 : cnt + ny*nx*i, : ) = ...
            -kron(eyenx, sys.C.matrices(:,:,i));
    end
    
    cnt = cnt + numel(sys.C.bfuncs) * ny*nx + numel(sys.D.bfuncs) * ny*nu; % Filling Di not necessary. This is null
    
    for i = 1:numel(sys.K.bfuncs) 
        Q( cnt + ny*nx*(i-1) +1 : cnt + ny*nx*i, : ) = ...
            kron(sys.K.matrices(:,:,i)', eyenx);
    end
    
    if sys.isX0
        cnt = cnt + numel(sys.K.bfuncs) * ny*nx;
        Q( cnt +1 : cnt + nx, : ) = kron(sys.x0',eyenx);
    end

    [Qt,~] = qr(Q);         % Notation of [2]
    P = Qt(:,nx^2+1:end);       % Find the orthogonal space with P'*P=I and P'*Q=0
    

        % ------------------------------------
        % 2. Compute the PE vector and its Jacobian
    [e,J] = LPVcore.numerics.lpvssest_errorWithGrad(data.InputData, rho, data.OutputData, sys);
    
    Jddlc = J*P;
    gk = Jddlc'*e;

    if ~isempty(find(isnan(Jddlc),1)) || ~isempty(find(isnan(e),1))
        idDisplayEstimationInfo('Progress', {' ', LPVcore.messageStr('LPVcore:general:bold',LPVcore.messageStr('LPVcore:lpvidss:NaNEorJac'))}, DispWind);
        sys = sysOld;
        break;
    end

        % ------------------------------------
        % 3-4. SVD Jacobian
	[UJ,SJ,VJ] = svd(Jddlc,0);
    
        % ------------------------------------
        % 5. Do loop
    loop = true;
    iterL = 1;
    maxIterL = options.SearchOptions.FindSearchDirMaxIter;
    
    while loop && iterL < maxIterL
            % 5.a
        sj = diag(SJ);
        r = find( (sj + lambda) >= gamma*(sj(1)+lambda) , 1, 'last');

            % 5.b
        qk = -VJ(:,1:r) / (diag(sj(1:r)+lambda./sj(1:r))) * UJ(:,1:r)'*e;
        
            % 5.c
        if( -qk'*gk >= nu_ * norm(qk)*norm(gk) )
            loop = false;
        else
            % 5.d-e
            if(gamma > epsilon)
                gamma = max(epsilon, 0.25*gamma);
            else
                lambda = max(lambdaMIN, 2*lambda);
            end
        end
        
        iterL = iterL+1;
    end
    
    if(iterL >= maxIterL)
        idDisplayEstimationInfo('Progress', LPVcore.messageStr('LPVcore:lpvidss:NoSearchDirection'), DispWind);
    end

	%% Compute step length
    
        % ------------------------------------
        % 6. Backstepping
    alpha = 1;
    
    VNo = sqrt(e'*e);
    systmp = theta2sys(theta+alpha*P*qk,sys);
	VNa = LPVcore.numerics.lpvssest_ell2loss(data.InputData, rho, data.OutputData, systmp, W);
    
	iterL = 1;
    maxIterL = options.SearchOptions.BackstepMaxIter;
    
    while VNa >= VNo + alpha*beta*qk'*gk  && iterL < maxIterL
        
        alpha = alpha*0.5;      % [3, Algorithm 6.3.5] upperbound. This ensures robustness
        
        systmp = theta2sys(theta+alpha*P*qk,sys);
        VNa = LPVcore.numerics.lpvssest_ell2loss(data.InputData, rho, data.OutputData, systmp, W);
        
        iterL = iterL+1;
    end
    
    
    if(iterL >= maxIterL)
        idDisplayEstimationInfo('Progress', LPVcore.messageStr('LPVcore:lpvidss:NoStepLength'), DispWind);
    end
    
     
        % ------------------------------------
        % 7-8. Update
    if alpha == 1
        gamma = max(epsilon,0.25*gamma);
        lambda = lambda/2;
    
    elseif alpha <= alphaMIN
        
        gamma = min(1,2*gamma);
    end  
    
        % ------------------------------------
        % 9. Update theta
    absChange = sum(abs( alpha*P*qk ))/nt;
    relChange = sum(abs( alpha*P*qk ))/min(sum(abs(theta)), sum(abs(theta+alpha*P*qk)));
    
    sysOld = sys;   % store old system
    
    theta = theta + alpha*P*qk;
    sys = systmp;   % allready computed
    
        % ------------------------------------
        % 10. update termination condition
	trmntncond = gk'/(Jddlc'*Jddlc + epsilon*eye(nt-nx^2))*gk;
    
    iter = iter +1;
    
    if Disp && DispWind.ActiveJavaWindow
        idDisplayEstimationInfo('Progress', sprintf(' %5d      %1.4e      %1.4e      %1.4e      %1.4e      %1.4e', ...
                        iter, VNa, trmntncond, alpha, absChange, relChange), DispWind);
        drawnow % update GUI handles to obtain correct DispWind.STOP
    end
    
end

    %% Post-processing: Save data in the lpvidss object
    
    sysinit.A = sys.A;
    sysinit.B = sys.B;
    sysinit.C = sys.C;
    sysinit.D = sys.D;
    sysinit.K = sys.K;
    if estInit
        sysinit.X0 = sys.x0;
    end


	%% Post-processing: create results report.
    
    yh = LPVcore.numerics.lpvssest_predict(data.InputData, rho, data.OutputData, sys);
    
    eh = (data.OutputData-yh);
    N = size(data,1);
    
    sysinit.NoiseVariance = eh'*eh/(N-1);
    
% 	ver = version('-release');
%     ver = str2double(ver(1:4));
    
%     if ver > 2017
%         sysinit.Report_ = createReport(sysinit.Report_, yh, sys.nt, data, options, ...
%             LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
%             ctrlMsgUtils.message('Ident:estimation:msgStatusPredictionFocusWithOptions','LPV PEM-SS', 'using: Enhanced Gauss Newton Method'));
%     else
%         sysinit.Report_ = createReport(sysinit.Report_, yh, sys.nt, data, options, ...
%             LPVcore.messageStr(['LPVcore:lpvidss:',options.SearchMethod]), ...
%             ctrlMsgUtils.message('Ident:estimation:msgStatusValue','LPV SS using: Enhanced Gauss Newton Method','prediction'));
%     end
%     
%      
%     if Disp && DispWind.ActiveJavaWindow
%         idpack.EstimProgress.displayResults(DispWind, sysinit.Report);
%         DispWind.STOP = true;
%     end
    
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


function sys = theta2sys(theta,sys)

    nx = sys.nx;
    nu = sys.nu;
    ny = sys.ny;
    count = 1;

    sys.A.matrices = reshape(theta(1:nx^2*sys.npsiA),nx,nx, sys.npsiA);
    count = count+nx^2*sys.npsiA;

    sys.B.matrices = reshape(theta(count:count+nx*nu*sys.npsiB-1),nx,nu, sys.npsiB);
    count = count+nx*nu*sys.npsiB;

    sys.C.matrices = reshape(theta(count:count+nx*ny*sys.npsiC-1),ny,nx, sys.npsiC);
    count = count+nx*ny*sys.npsiC;

    sys.D.matrices = reshape(theta(count:count+ny*nu*sys.npsiD-1),ny,nu, sys.npsiD);
    count = count+ny*nu*sys.npsiD;

    sys.K.matrices = reshape(theta(count:count+ny*nx*sys.npsiK-1),nx,ny, sys.npsiK);
    count = count+ny*nx*sys.npsiK;

    if sys.isX0
        sys.x0 = theta(count:count+nx-1);
    end

end



