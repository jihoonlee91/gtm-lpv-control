function [sys, ic] = lpvobfest(data, template_sys, options)
%LPVOBFEST Estimation of LPVIDOBF system
%
%   Syntax:
%       sys = lpvobfest(data, template_sys)
%       sys = lpvobfest(data, template_sys, options)
%       [sys, ic] = lpvobfest(___)
%
%   Inputs:
%       data: lpviddata object
%       template_sys: lpvidobf object that indicates the structure of the
%           model to be identified.
%       options: lpvobfestOptions object
%
%   Outputs:
%       sys: estimated lpvidobf object
%       ic: estimated initial conditions, specified as a vector of doubles
%

%% Input validation
assert(isa(data, 'lpviddata'), '''data'' must be an lpviddata object');
assert(isa(template_sys, 'lpvidobf'), '''template_sys'' must be an lpvidobf object');
if nargin <= 2
    options = lpvobfestOptions;
end
assert(isa(options, 'lpvobfestOptions'), ...
    '''options'' must be an lpvobfestOptions object');

%% Identification
assert(isempty(template_sys.Wf) || isempty(template_sys.Hf), ...
    'Hammerstein-Wiener model structure (Wf and Hf non-empty) are not supported');

% Case 1: Wiener
if isempty(template_sys.Hf)
    % TODO: initial state estimation (using e.g. pseudo-lin regr)
    if isequal(options.InitialState, 'estimate')
        error('The ''InitialState'' option cannot be set to ''estimate'' for Wiener models');
    end
    [sys, ic] = lpvobfestwiener_(data, template_sys, options);
elseif isempty(template_sys.Wf)
    % Case 2: Hammerstein (incl. initial state estimation, easier than Wiener)
    [sys, ic] = lpvobfesthammerstein_(data, template_sys, options);
end
% Extension to MIMO: approach #1 from 2.3.6 of the book

%% Update Report
sys.X0 = ic;
[~, fit] = compare(data, sys);
sys.Report.Status = 'Estimated using LPVOBFEST';
sys.Report.Method = 'lpvobfest';
sys.Report.OptionsUsed = options;
sys.Report.DataUsed = data;
% Prediction and simulation fit are the same for OE models
sys.Report.Fit.PredictionFit = fit;
sys.Report.Fit.SimulationFit = fit;

end
