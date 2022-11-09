function [est_sys, ic] = lpvobfestwiener_(data, template_sys, opts)

% Algorithm 9.3 of "Modeling and Identification of Linear Parameter-Varying
% Systems" (2010), R. Toth

% Step 3b
% Set of basis functions
Phi = template_sys.M;
[~, ~, yt] = lsim(Phi, data.u);

% Step 4b
% N_{d}-by-n_{psi} matrix of evaluated basis functions
% TODO: check orthogonality of Psi as stated in Step 4b?
Psi = feval(template_sys.Wf, data.p, 'basis');

% Step 5b
Nd = size(yt, 1);
% (9.42)
gamma = NaN(Nd, (size(yt, 2) + size(data.u, 2)) * size(Psi, 2));
for k=1:size(data.y, 1)
    gamma(k, :) = kron([yt(k, :), data.u(k, :)], Psi(k, :));
end
Gamma = gamma;
Y = data.y;

% (9.45) (TODO: with support for non-free parameters)

% The order of theta in (9.45) is as follows:
%   theta = [thetaC; thetaD]
%   thetaC = [thetaC_state1; ...; thetaC_stateN]
%   thetaC_statei = [thetaC_statei_bfunc1; ...; thetaC_statei_bfuncM]
% The order of the parameter vector ("theta_r") in LPVcore is as follows:
%   theta = [thetaC; thetaD]
%   thetaC = [thetaC_bfunc1; ...; thetaC_bfuncM]
%   thetaC_bfunci = [thetaC_bfunci_state1; ...; thetaC_bfunci_stateN]
%
theta = Gamma \ Y;

% Reshape theta
% Loop over basis functions
theta_r = [];
for i=1:size(Psi, 2)
    idx=(i:size(Psi, 2):numel(theta));
    theta_r = [theta_r, theta(idx)]; %#ok<AGROW> 
end
theta_r_C = theta_r(1:end-1, :);
theta_r_D = theta_r(end, :);
theta_r = [theta_r_C(:); theta_r_D(:)];

% Assign free parameters
est_sys = template_sys;
est_sys = setpvec(est_sys, theta_r);

% TODO: add initial conditions to algorithm
% Get initial condition from options set
ic = getInitialState(opts, template_sys);

end

