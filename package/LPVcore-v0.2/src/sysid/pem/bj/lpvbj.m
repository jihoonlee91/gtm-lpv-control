function sys = lpvbj(data, template_sys, options)
%LPVBJ Estimating an LPV-BJ model from data    
%
%   Syntax:
%       sys = lpvbj(data, template_sys)
%       sys = lpvbj(data, template_sys, options)
%
%   Inputs:
%       data: lpviddata object.
%       template_sys: template lpvidpoly object. The structure will be
%           used, but not the coefficient values.
%       options: lpvbjOptions for controlling the identification
%           parameters.
%
% See also PMATRIX, LPVBJOPTIONS
narginchk(2, 3);

assert(isa(data, 'lpviddata'), ...
    'Data must be of type lpviddata.');
assert(isa(template_sys, 'lpvidpoly'), ...
    'Template system must be an lpvidpoly object.');
assert(isbj(template_sys), ...
    'Template system must be an LPV-BJ model structure.');
[~,~,~,nu] = size(data);
assert(nu > 0, 'Data must have at least 1 input');

if nargin <= 2
    options = lpvbjOptions;
end

assert(isa(options, 'lpvbjOptions'), 'options must be an instance of lpvbjOptions');

sys = lpvpolyest(data,template_sys,options);

end