function sys = lpvsid(data, template_sys, options)
%LPVSID Estimate LPV state-space model using subspace method
%
%   Syntax:
%       sys = lpvsid(data, template_sys)
%       sys = lpvsid(data, template_sys, options)
%
%   Inputs:
%       data: lpviddata object containing identification dataset
%
%       template_sys: lpvidss object representing a template model
%           structure from which scheduling dependence is extracted.
%
%       options: <a href="matlab:doc lpvsidOptions">lpvsidOptions</a> object to specify the options for the
%           estimation.
%
%   The lpvsid command currently supports the LPV-PBSIDopt algorithm. For
%   more information, see the following references:
%
%   Reference: [1] "Subspace identification of Bilinear and LPV systems for
%       open- and closed-loop data" (J-W van Wingerden, M. Verhaegen, 2009)
%
%   Additional derivations: [2] "LPVPBSID_opt additional documentation",
%       www.lpvcore.net
%
%   See also LPVSIDOPTIONS, LPVSSEST, LPVIDSS
%

%% Input validation
narginchk(2, 3);

assert(isa(data, 'lpviddata'), '''data'' must be an lpviddata object');
assert(isa(template_sys, 'lpvidss'), '''template_sys'' must be an lpvidss object');
if nargin <= 2
    options = lpvsidOptions;
end
assert(isa(options, 'lpvsidOptions'), '''options'' must be an lpvsidOptions object');

%% Display
gui = progressgui(template_sys, data, options);

%% Algorithm
sys = lpvpbsidopt_(data, template_sys, options, gui);

end

