function [e,J] = lpvssest_errorWithGrad(u, rho, y, sys)

% We assume that either (A, B, K) is nonempty (existance of state dynamics)
% and C nonempty (state is observed). Else we do not require a state. This
% implies that AT = A-K*C always exists and C always exists

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

nx = sys.nx;
ny = sys.ny;
nu = sys.nu;
nt = sys.nt;     % Amount of parameters to be estimated

u = u';
y = y';

N = size(u,2);

xh  = zeros(nx,N+1);
if sys.isX0; xh(:,1) = sys.x0; end  % Set initial condition
yh  = zeros(ny,N);

dxht1 = zeros(nx,nt);   % Old dxht
J     = zeros(ny*N,nt);

    % precompute the matrices over the scheduling trajectory
[AFeval, rhoA] = feval(sys.A,rho); 

[BFeval, rhoB] = feval(sys.B,rho); 

[CFeval, rhoC] = feval(sys.C,rho);

[DFeval, rhoD] = feval(sys.D,rho); 

[KFeval, rhoK] = feval(sys.K,rho); 

% if sys.isA == 0; isAnemp = false;
% else;            isAnemp = true; end
% if sys.isB == 0; isBnemp = false;
% else;            isBnemp = true; end
% C always exists
isDnemp = ~isempty(sys.D);
isKnemp = ~isempty(sys.K);

if ~isempty(sys.A) && ~isempty(sys.C) && ~isempty(sys.K)
    ATFeval = zeros(nx, nx, N);
    for i = 1:N; ATFeval(:,:,i) = AFeval(:,:,i) - KFeval(:,:,i)*CFeval(:,:,i); end
elseif ~isempty(sys.C) && ~isempty(sys.K)
    ATFeval = zeros(nx, nx, N);
    for i = 1:N; ATFeval(:,:,i) = - KFeval(:,:,i)*CFeval(:,:,i); end
else  % sys.isA ~= 0
    ATFeval = AFeval;
end

if ~isempty(sys.B) && ~isempty(sys.D) && ~isempty(sys.K)
    isBTnemp = true;
    BTFeval = zeros(nx, nu, N);
    for i = 1:N; BTFeval(:,:,i) = BFeval(:,:,i) - KFeval(:,:,i)*DFeval(:,:,i); end
elseif ~isempty(sys.D) && ~isempty(sys.K)
    isBTnemp = true;
    BTFeval = zeros(nx, nu, N);
    for i = 1:N; BTFeval(:,:,i) = - KFeval(:,:,i)*DFeval(:,:,i); end
elseif ~isempty(sys.B)
    isBTnemp = true;
    BTFeval = BFeval;
else
    isBTnemp = false;
end


% Auxiliary variables
% vzx = zeros(nx,1);
% vzy = zeros(ny,1);
eyenx = eye(nx);
eyeny = eye(ny);

npsiA = numel(sys.A.bfuncs);
npsiB = numel(sys.B.bfuncs);
npsiC = numel(sys.C.bfuncs);
npsiD = numel(sys.D.bfuncs);
npsiK = numel(sys.K.bfuncs);


for t = 1:N

    xh(:,t+1) = ATFeval(:,:,t)*xh(:,t);
    if isBTnemp; xh(:,t+1) = xh(:,t+1)+  BTFeval(:,:,t)*u(:,t); end
    if isKnemp;  xh(:,t+1) = xh(:,t+1)+  KFeval(:,:,t)*y(:,t);  end
    
    yh(:,t)   = CFeval(:,:,t)*xh(:,t);
    if isDnemp;  yh(:,t)   = yh(:,t) + DFeval(:,:,t)*u(:,t);    end
        
    count  = 1;
    
    %% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. a^k_{i,j}
    

%     for k = 1:sys.npsiA     % If for loop passes isAnemp = true
%         for j = 1:nx
%             for i = 1:nx
%                 
%                 xtmp      = vzx;
%                 xtmp(i,1) = rhoA(t,k)*xh(j,t);
%                     
%                 dxht(:, count) = ATFeval(:,:,t)*dxht1(:,count) + xtmp;
%                 
%                 J(ny*(t-1)+1:ny*t,count) = CFeval(:,:,t)*dxht1(:,count);
%                 
%                 count = count+1;
%             end
%         end
%     end

    dxht = ATFeval(:,:,t)*dxht1;
    J(ny*(t-1)+1:ny*t,:) = CFeval(:,:,t)*dxht1;
    
    for k = 1:npsiA     % If for loop passes isAnemp = true
        for j = 1:nx
            
            rng = count:count+nx-1;
            
            dxht(:, rng) = dxht(:, rng) + rhoA(t,k)*xh(j,t) * eyenx;
            count = count+nx;
            
        end
    end

    %% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. b^k_{i,j}
    
%     for k = 1:sys.npsiB
%         for j = 1:nu
%             for i = 1:nx
%                 utmp      = vzx;
%                 utmp(i,1) = rhoB(t,k)*u(j,t);
% 
%                 dxht(:, count) = ATFeval(:,:,t)*dxht1(:,count) + utmp;
%                 J(ny*(t-1)+1:ny*t,count) = CFeval(:,:,t)*dxht1(:,count);
% 
%                 count = count+1;
%             end
%         end
%     end    

    for k = 1:npsiB
        for j = 1:nu
            rng = count:count+nx-1;
            dxht(:, rng) = dxht(:, rng) + rhoB(t,k)*u(j,t) * eyenx;
            count = count+nx;
        end
    end

	%% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. c^k_{i,j}
    
%     for k = 1:sys.npsiC
%         for j = 1:nx
%             for i = 1:ny
%                 xtmp      = vzy;
%                 xtmp(i,1) = rhoC(t,k)*xh(j,t);
%                 
%                 dxht(:, count) = ATFeval(:,:,t)*dxht1(:,count) - KFeval(:,i,t)*rhoC(t,k)*xh(j,t);
%                 J(ny*(t-1)+1:ny*t,count) = xtmp+CFeval(:,:,t)*dxht1(:,count);
% 
%                 count = count+1;
%             end
%         end
%     end

    for k = 1:npsiC
        for j = 1:nx
            
            rng = count:count+ny-1;
            
            if isKnemp; dxht(:, rng ) = dxht(:, rng) - KFeval(:,:,t)*rhoC(t,k)*xh(j,t); end
            J(ny*(t-1)+1:ny*t, rng) = J(ny*(t-1)+1:ny*t, rng) + rhoC(t,k)*xh(j,t) * eyeny;
            
            count = count+ny;
            
        end
    end

        
    
	%% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. d^k_{i,j}
    

%     for k = 1:sys.npsiD
%         for j = 1:nu
%             for i = 1:ny
%                 utmp      = vzy;
%                 utmp(i,1) = rhoD(t,k)*u(j,t);
% 
%                 dxht(:, count) = ATFeval(:,:,t)*dxht1(:,count) - KFeval(:,i,t)*rhoD(t,k)*u(j,t);
%                 J(ny*(t-1)+1:ny*t,count) = utmp+CFeval(:,:,t)*dxht1(:,count);
% 
%                 count = count+1;
%             end
%         end
%     end
    
    for k = 1:npsiD
        for j = 1:nu
            
            rng = count:count+ny-1;
            
            if isKnemp; dxht(:, rng) = dxht(:, rng) - KFeval(:,:,t)*rhoD(t,k)*u(j,t); end
            J(ny*(t-1)+1:ny*t,rng) = J(ny*(t-1)+1:ny*t,rng) + rhoD(t,k)*u(j,t) * eyeny;
            
            count = count+ny;
            

        end
    end

	%% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. k_{i,j}
    
%     for k = 1:sys.npsiK
%         for j = 1:ny
%             for i = 1:nx
% 
%                 xtmp      = vzx;
%                 xtmp(i,1) = rhoK(t,k)*CFeval(j,:,t)*xh(:,t);
% 
%                 utmp      = vzx;
%                 if isDnemp; utmp(i,1) = rhoK(t,k)*DFeval(j,:,t)*u(:,t); end
% 
%                 ytmp      = vzx;
%                 ytmp(i,1) = rhoK(t,k)*y(j,t);
% 
%                 dxht(:, count) = ATFeval(:,:,t)*dxht1(:,count) - xtmp - utmp + ytmp;
%                 J(ny*(t-1)+1:ny*t,count) = CFeval(:,:,t)*dxht1(:,count);
% 
%                 count = count+1;
%             end
%         end
%     end
    
    for k = 1:npsiK
        for j = 1:ny
            
            rng = count:count+nx-1;
            
            if isDnemp
                dxht(:, rng) = dxht(:, rng) + rhoK(t,k)*(y(j,t) - CFeval(j,:,t)*xh(:,t)  - DFeval(j,:,t)*u(:,t)) * eyenx;
            else
                dxht(:, rng) = dxht(:, rng) + rhoK(t,k)*(y(j,t) - CFeval(j,:,t)*xh(:,t)) * eyenx;
            end
            
            count = count+nx;
            
        end
    end

    %% ------------------------------------------------------------
    % Compute the gradient and derivative of xht w.r.t. x_{i}
    
    if t == 1 && sys.isX0
        rng = count:count+nx-1;
        dxht(:, rng) = ATFeval(:,:,1);
    end
    
    dxht1 = dxht;
    
end

    J = -J;
        % Should be outside of the loop
    e = y(:)-yh(:);     % create one long vector

end