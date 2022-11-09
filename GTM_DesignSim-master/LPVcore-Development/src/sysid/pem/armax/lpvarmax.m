function sys = lpvarmax(data, template_sys, options)
%LPVARMAX Extimating an LPV-ARMAX model from data    
%
%   Syntax:
%       sys = lpvarmax(data, template_sys)
%       sys = lpvarmax(data, template_sys, options)
%
%   Inputs:
%       data: lpviddata object.
%       template_sys: template lpvidpoly object. The structure will be
%           used, but not the coefficient values.
%       options: lpvarmaxOptions for controlling the identification
%           parameters.
%
% See also PMATRIX, LPVARMAXOPTIONS
narginchk(2, 3);

assert(isa(data, 'lpviddata'), ...
    'Data must be of type lpviddata.');
assert(isa(template_sys, 'lpvidpoly'), ...
    'Template system must be an lpvidpoly object.');
assert(isarmax(template_sys), ...
    'Template system must be an LPV-ARMAX model structure.');
[~,~,~,nu] = size(data);
assert(nu > 0, 'Data must have at least 1 input');

if nargin <= 2
    options = lpvarmaxOptions;
end

assert(isa(options, 'lpvarmaxOptions'), 'options must be an instance of lpvarmaxOptions');

sys = lpvpolyest(data, template_sys, options);

end