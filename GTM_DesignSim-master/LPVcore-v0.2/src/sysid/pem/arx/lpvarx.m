function sys = lpvarx(data, template_sys, options, regulOptions)
%LPVARX Extimating an LPV-ARX model from data    
%
%   Syntax:
%       sys = lpvarx(data, template_sys)
%       sys = lpvarx(data, template_sys, options)
%       sys = lpvarx(data, template_sys, options, regulOptions)
%
%   Inputs:
%       data: lpviddata object.
%       template_sys: template lpvidpoly object. The structure will be
%           used, but not the coefficient values.
%       options: lpvarxOptions for controlling the identification
%           parameters.
%       regulOptions: lpvarxRegulOptions for configuring the process of
%           automatic determination of regularization parameters (uses
%           lpvarxRegul in the background).
%
% See also PMATRIX, LPVARXOPTIONS, LPVARXREGUL, LPVARXREGULOPTIONS
narginchk(2, 4);

assert(isa(data, 'lpviddata'), ...
    'Data must be of type lpviddata.');
assert(isa(template_sys, 'lpvidpoly'), ...
    'Template system must be an lpvidpoly object.');
assert(isarx(template_sys), ...
    'Template system must be an LPV-ARX model structure.');
[~,~,~,nu] = size(data);
assert(nu > 0, 'Data must have at least 1 input');

if nargin <= 2
    options = lpvarxOptions;
end
if nargin <= 3
    regulOptions = [];
end

sys = arx_(template_sys, data, options, regulOptions);

end