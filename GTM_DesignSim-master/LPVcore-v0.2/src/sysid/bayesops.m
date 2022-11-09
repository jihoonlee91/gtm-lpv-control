function [pNext, nugapEst, results] = bayesops(G, p, varargin)
%BAYESOPS Operating Point Selection based on Bayesian Optimiation
%   BAYESOPS is used in the local identification of LPV systems to
%   determine a candidate operating point to include in the identification.
%   It is based on the Gaussian Process modelling of the nu-gap metric over
%   the operating range, and then proposing the sample point where the
%   uncertainty is highest.
%
%   Syntax:
%       [pNext, nugapEst, results] = bayesops(G, p, Name, Value)
%
%   Inputs:
%       G: cell array of LTI systems
%       p: cell array of frozen scheduling values at which G were determined
%
%   Outputs:
%       pNext: (1-by-np vector) next operating point.
%       nugapEst: Gaussian Process-based estimate of the nu-gap metric.
%       results: results object from call to BAYESOPT.
%
%   Name-Value pairs:
%       See BAYESOPT. They are directly passed to the underlying BAYESOPT.
%

if ~iscell(G); G = {G}; end
if ~iscell(p); p = {p}; end
if numel(G) ~= numel(p)
    error('The number of elements of G and p must be the same.');
end
N = numel(p);
for i=1:N
    if size(p{i}, 1) ~= 1
        p{i} = p{i}';
    end
    if ~isstable(G{i})
        error(['System G{', int2str(i), '} is unstable.']);
    end
end
np = numel(p{1});

% Construct GP model
X = NaN(N^2, 2 * np);
y = NaN(N^2, 1);
for i=1:N
    for j=1:N
        X((i-1)*N+j, :) = [p{i}, p{j}];
        [~, y((i-1)*N+j)] = gapmetric(G{i}, G{j});
    end
end

gp = fitrgp(X, y, ...
        'KernelFunction', @(Xm, Xn, theta) nugapse(Xm, Xn, theta), ...
        'KernelParameters', [10, 1]);

% Call BAYESOPT
vars = [];
for i=1:2*np
    vars = [vars; optimizableVariable(['x', int2str(i)], [-1, 1])]; %#ok<AGROW>
end

nugapEst = @(x, y) predict(gp, [x, y]);

results = bayesopt(@(x) -acquisitionfnc(gp, x), vars, ...
    'MaxObjectiveEvaluations', 10, ...
    'PlotFcn', {}, ...
    varargin{:});

% Get two candidates for pNext and select one with highest min. distance to
% other points
pCandidates = results.XAtMinObjective{1, :};
pCandidate1 = pCandidates(1:np);
pCandidate2 = pCandidates(np+1:end);
minDist1 = Inf;
minDist2 = Inf;
for k=1:N
    minDist1 = min(minDist1, norm(p{k} - pCandidate1));
    minDist2 = min(minDist2, norm(p{k} - pCandidate2));
end
if minDist1 > minDist2
    pNext = pCandidate1;
else
    pNext = pCandidate2;
end

end

function A = acquisitionfnc(gp, x)
    % ACQUISITIONFNC Acquisition function for operating point selection
    %   Sampling occurs at the point where the uncertainty is largest.
    [~, A] = predict(gp, x);
end

function Kmn = nugapse(Xm, Xn, theta)
    % NUGAPSE Squared Exponential Kernel with modifications for modeling
    % the nu-gap metric.
    
    % Swap columns of Xm to enforce symmetry.
    % See https://www.cs.toronto.edu/~duvenaud/cookbook/
    d = size(Xm, 2) / 2;
    XmSwapped = Xm * [zeros(d), eye(d); eye(d), zeros(d)];
    
    Pm = Xm(:, 1:d);
    Qm = Xm(:, d+1:end);
    Dm = vecnorm(Pm - Qm, 2, 2); % m-by-1
    
    Pn = Xn(:, 1:d);
    Qn = Xn(:, d+1:end);
    Dn = vecnorm(Pn - Qn, 2, 2); % n-by-1
    
    D = Dm * Dn';               % m-by-n
    
    Kmn = 0.5 * D.^(1/2) .* (squaredexponential(Xm, Xn, theta) + ...
        squaredexponential(XmSwapped, Xn, theta));
end

function Kmn = squaredexponential(Xm, Xn, theta)
    m = size(Xm, 1);
    n = size(Xn, 1);
    Kmn = NaN(m, n);
    sigmal = exp(theta(1));
    sigmaf = exp(theta(2));
    for i=1:m
        for j=1:n
            xi = Xm(i, :)';
            xj = Xn(j, :)';
            Kmn(i, j) = sigmaf^2 * exp(-1 / (2 * sigmal) * ...
                (xi - xj)' * (xi - xj));
        end
    end
end

