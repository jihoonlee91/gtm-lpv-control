function solverError = lpvnormErrorCheck(diagnostics,type)
%LPVNORMERRORCHECK Checks if LMI optimization problem for LPV analysis is
%correctly solved based on the diagnostic information and generates error
%message if that is not the case.
%
%   Syntax:
%       solverError = lpvnormErrorCheck(diagnostics,type)
%
%   Inputs:
%       diagnostics (struct): Diagnostics information, assumed to be the
%           output of YALMIP optimize function. see <a href=
%           "matlab:help optimize">optimize</a>.
%       type (char): Character array corresponding to the performance
%           metric that is analyzed.
%
%   Outputs:
%       solverError: Boolean indicating if optimization problem for
%           analysis was correctly solved (1) or not (0).
%
%   See also LPVNORM.
%
if strcmpi(type,'passive')
    errorText = 'Could not compute if the system is passive.\n';
else
    errorText = sprintf('Could not compute the %s-gain.\n',lower(type));
end
errorMessage = [sprintf(errorText),...
    sprintf('Solver returned error code %i: %s. ',diagnostics.problem,...
    diagnostics.info)];
if diagnostics.problem == 0
    solverError = false;
else
    warning(errorMessage);
    solverError = true;
end
end