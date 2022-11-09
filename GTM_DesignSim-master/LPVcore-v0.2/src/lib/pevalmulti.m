function varargout = pevalmulti(varargin)
%PEVALMULTI Basis function evaluation for multiple PMATRIX objects
%
% When dynamic dependency is introduced in the basis functions, 
% then calculating the extended scheduling variable rho from the 
% scheduling variable p leads to truncation in discrete-time, 
% since shifted versions of p are unavailable near the edges of 
% the data. For continuous-time, truncation may happen in 
% the future (currently, we use the built-in GRADIENT function which preserves 
% dimension of the input but uses a different numerical differentiation 
% scheme at the edges, which may be undesirable).
%
% The basis functions of a single pmatrix object can be 
% calculated using the PEVAL function as follows:
%
%   [A, psi] = peval(pmatrix, p)
% 
% However, this leads to issues when dealing with multiple 
% pmatrix objects, each with their own dynamic dependency and, 
% thus, their own truncation windows. To combat this issue, 
% the function PEVALMULTI is created that accepts multiple pmatrix 
% objects as inputs, a trajectory for p and timestamps t (optional). 
% It returns properly and uniformly truncated trajectories for 
% all the basis functions, p and t. The truncation window is, 
% naturally, determined by the pmatrix object with the largest time 
% shift (DT) or highest-order derivative (CT) in its scheduling map.
%
% Syntax:
%
%   [psi1, ..., psiN] = pevalmulti(A1, ..., AN, p)
%   [psi1, ..., psiN, p, t] = pevalmulti(A1, ..., AN, p, t)
%
% Inputs:
% 
%   A1, ..., AN: pmatrix objects with arbitrary scheduling dependence (dynamic or static). 
%   p: trajectory of the scheduling variable (not the extended scheduling variable).
%   t: optional vector of timestamps for the specified trajectory of p.
% 
% Outputs:
% 
%   psi1, ..., psiN: N-by-nrho numeric matrices representing the trajectories 
%       of the basis functions for each input pmatrix object. The number of 
%       rows, N, is upper bounded by the length of the input trajectory p.
%   p: truncated version of the input scheduling trajectory.
%   t: numeric vector of timestamps corresponding to the basis functions. 
%       This is a truncated version of the input.
% 

%% Input validation
narginchk(2, Inf);

% First input arguments are pmatrix objects, stored in cell array A
A = {};
% Ensure that the scheduling signal is consistent by comparing the name of
% the scheduling variable using this auxiliary variable.
pName = NaN;
for i=1:nargin
    if isa(varargin{i}, 'pmatrix')
        if i == nargin
            error('Trajectory of scheduling variable must be specified');
        end
        A = [A, varargin(i)]; %#ok<AGROW>
        % Check consistency of the scheduling signal based on name
        if ~iscell(pName)
            % Initialize pName
            pName = varargin{i}.timemap.Name;
        else
            pNameNew = varargin{i}.timemap.Name;
            assert(numel(pName) == numel(pNameNew) && ...
                all(strcmp(pName, pNameNew)), ...
                'Input pmatrix objects must have the same scheduling vector');
        end
    else
        % Remaining arguments are the trajectory of p
        % and optionally timestamps t.
        if nargin > i+1
            error('Too many input arguments');
        end
        p = varargin{i};
        assert(isnumeric(p) && ismatrix(p), ...
            'p must be a numeric trajectory');
        if numel(varargin) <= i
            t = (1:1:size(p, 1))';
        else
            t = varargin{i+1};
        end
        assert(isnumeric(t) && isvector(t) && ...
            numel(t) == size(p, 1) && ...
            all(diff(t) > 0) && ...
            norm(diff(t) - mean(diff(t))) / numel(t) < 10 * eps, ...
            't must be an increasing, equidistant, numeric vector with the same number of elements as p has rows');
        % Extract sampling time from t
        if numel(t) > 1
            Ts = t(2) - t(1);
        else
            Ts = 1;
        end
        break
    end
end
% Check dimension of p
assert(numel(A{1}.timemap.Name) == size(p, 2), ...
    'Number of columns of p must be equal to dimension of scheduling signal');
% Check number of output arguments
nargoutchk(0, numel(A)+2);

%% Calculate basis functions
psi = cell(1, numel(A));
% Max. truncation at the start among all pmatrix objects
globalTruncStart = 0;
% Max. truncation at the end among all pmatrix objects
globalTruncEnd = 0;
for i=1:numel(A)
    % Determine truncation window applied to this pmatrix object by PEVAL
    if strcmpi(A{i}.timemap.Domain, 'dt')
        truncStart = abs(min(0, min(A{i}.timemap.Map)));
        truncEnd = max(0, max(A{i}.timemap.Map));
    else
        truncStart = 0; truncEnd = 0;
    end
    % Update global truncation window
    if truncStart > globalTruncStart; globalTruncStart = truncStart; end
    if truncEnd > globalTruncEnd; globalTruncEnd = truncEnd; end
    % Calculate un-truncated (raw) basis functions
    [~, psiRaw] = peval(A{i}, p, Ts);
    % Pad with NaNs to ensure all psi's have same size
    psi{i} = [ NaN(truncStart, size(psiRaw, 2)); ...
        psiRaw; NaN(truncEnd, size(psiRaw, 2))];
end

%% Truncate NaN-padded basis functions, scheduling trajectory and time vector
for i=1:numel(psi)
    psiRaw = psi{i};
    psi{i} = psiRaw(globalTruncStart+1:end-globalTruncEnd, :);
end
t = t(globalTruncStart+1:end-globalTruncEnd, :);
p = p(globalTruncStart+1:end-globalTruncEnd, :);

%% Return
if nargout == numel(psi) || nargout == 0
    varargout = psi;
elseif nargout == numel(psi) + 1
    varargout = [psi, {p}];
elseif nargout == numel(psi) + 2
    varargout = [psi, {p}, {t}];
end

end

