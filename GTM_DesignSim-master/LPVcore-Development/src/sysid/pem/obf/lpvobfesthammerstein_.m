function [est_sys, x0] = lpvobfesthammerstein_(data, template_sys, opts)

% Algorithm 9.3 of "Modeling and Identification of Linear Parameter-Varying
% Systems" (2010), R. Toth with modifications (9.47)-(9.50) for Hammerstein
% case.

Nd = size(data.y, 1);
ng = numel(template_sys.v);
ne = template_sys.ne - 1;
u = data.u;

% Set of orthogonal basis functions
Phi = template_sys.M;
% Calculate h (9.49), which can be expressed as:
%   h(0) = 0
%   h(1) = C
%   h(2) = C * A
%   h(3) = ...
%   h(Nd-1) = C * A^(Nd - 1)
% We calculate "h_ext" with 1 additional sample, which is used for the
% initial state estimation later.
Phi_h = ss(Phi.A.', Phi.C.', eye(size(Phi.A, 1)), [], Phi.Ts);
h_ext = impulse(Phi_h, Nd);
h = h_ext(1:end-1, :);    % For forced response
h_x0 = h_ext(2:end, :); % For free response

% N_{d}-by-n_{psi} matrix of evaluated basis functions
% TODO: check orthogonality of Psi as stated in Step 4b?
Psi = feval(template_sys.Hf, data.p, 'basis');
npsi = size(Psi, 2);

% (9.50)
H = zeros(Nd, ng * (ne + 1), Nd);
for i=1:Nd
    H(i:end, :, i) = h(1:Nd-i+1, :);
end

Hhat = NaN(Nd, ng * (ne + 1), npsi);
for l=1:npsi
    Hhat(:, :, l) = H(:, :, 1) * u(1) * Psi(1, l);
    for k=2:Nd
        Hhat(:, :, l) = Hhat(:, :, l) + ...
            H(:, :, k) * u(k) * Psi(k, l);
    end
end

% Form 2D Gamma from 3D Hhat (just above (9.50))
Gamma = reshape(Hhat, Nd, []);

% Add direct feedthrough term (9.51) in last column
gamma = NaN(Nd, size(data.u, 2) * size(Psi, 2));
for k=1:Nd
    gamma(k, :) = kron(data.u(k, :), Psi(k, :));
end
Gamma = [Gamma, gamma];

% Add initial state in last columns.
%   Y =  Y_{forced}    +    Y_{free}
%   Y = Gamma * theta  +    h_x0 * x0
%   Y = [Gamma, h_x0] * [theta; x0]

Y = data.y;

if isequal(opts.InitialState, 'estimate')
    Y_est = Y;
    Gamma_est = [Gamma, h_x0];
else
    x0 = getInitialState(opts, template_sys);
    Y_est = Y - h_x0 * x0;
    Gamma_est = Gamma;
end

% (9.45) (with support for non-free parameters and initial state estimation)

% The order of theta in (9.45) (for HAMMERSTEIN models) is as follows:
%   theta = [thetaC; thetaD]
%   thetaC = [thetaC_bfunc1; ...; thetaC_bfuncM]
%   thetaC_bfunci = [thetaC_bfunci_state1; ...; thetaC_bfunci_stateN]
% The order of the parameter vector ("theta_r") in LPVcore is the same.
theta_est = Gamma_est \ Y_est;

theta = theta_est(1:nparams(template_sys, 'free'));
if isequal(opts.InitialState, 'estimate')
    x0 = theta_est(nparams(template_sys, 'free')+1:end);
end

% Assign free parameters
est_sys = template_sys;
est_sys = setpvec(est_sys, theta);

end



