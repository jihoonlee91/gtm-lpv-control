function [Lambda, R, optionsRegul, fval] = regul_LS(~, Rd1, Rd2, Sigma, N, optionsRegul)
%REGUL_LS Finding the regularization parameter Lambda for the following
%   least-squares problem:
%
%       min_theta  || Ynt - Phit * theta ||^2 + Lambda || theta ||^2
%
%   using emprical Bayesian marginal likelihood regression.
%
%   [Lambda, R, optionsRegul] = regul_LS(~, Rd1, Rd2, Sigma_, N, optionsRegul)
%
%   
%   where 
%        Sigma_ = chol(Sigma,'lower');       % Sigma is noise variance
%
%               % Filtering data with inverse noise variance
%          Ynt  = Sigma_ \ Yn;   Ynt = Ynt(:);  
%          Phit = kron(Phi, Sigma_ \ eye(ny) );  
%
%            Rd = triu(qr([Phit Ynt]));
%
%           Rd1 = Rd(1:npar,1:npar);
%           Rd2 = Rd(1:npar,npar+1);
%   

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    npar = size(Rd1,1);
    Optim = ~strcmp(optionsRegul.Advanced.SearchMethod,'gn');
    
    if Optim
       % Note:  check if Optimization Toolbox license ios available before
       % using 'active-set',  'interior-point'.
       alg = {'trust-region-reflective','active-set', 'interior-point','sqp'};
       fminconoptions = optimset('GradObj','on','Hessian','on','Hessian','user-supplied',...
          'Algorithm',alg{1},'Display','off');  
      
       [alpha, fval] = idnlcidsh(@(alpha) localHypermarginal(alpha, Sigma, Rd1, Rd2, N, npar),1,[],[],[],[],1e-40,inf,[],fminconoptions);
      
    else
       [alpha, fval] = idGNMinimize(@(alpha) localHypermarginal(alpha, Sigma, Rd1, Rd2, N, npar), 1, 1e-40, inf);
    end
    
    Lambda = 1/alpha;   % Note that Lambda is 1/alpha
    R = eye(npar);      % Note that R = inv(L1*L1')
end






function [f, g, H] = localHypermarginal(alpha, L2, Rd1, Rd2, N, n)
%LOCALHYPERMARGINAL Evaluate the marginal likelihood of p(Y | alpha).
%


L = sqrt(alpha)*eye(n);

    % Compute second factoruzation
% [Qc, Rc] = qr([Rd1*L Rd2;eye(n),zeros(n,1)]);
Rc = triu(qr([Rd1*L,Rd2;eye(n),zeros(n,1)]));

R1 = Rc(1:n,1:n);

r = Rc(n+1,n+1);

    % Compute the function evaluation
% f = 2*N*det(L2) + log(det(R1)^2) + r^2;

% f = 2*N*det(L2) + log(prod(diag(R1))^2) + r^2;

f = 2*N*det(L2) + 2*sum(log(abs(diag(R1)))) + r^2;  % less troubles with inf


if nargout > 1 % gradient required

    Li = eye(size(L,1)) /L;
    R2 = Rc(1:n,end);
    
    X1 = Li'*Li - Li' / (R1'*R1) * Li;
    X2 = Li' / R1 * (R2 * R2') / (R1') * Li;
    
    g = trace( (X1-X2) );
    
    if nargout > 2 % Hessian required
        H = trace((X1*X2 - (X1-X2)*X1) );
    end

end


end

