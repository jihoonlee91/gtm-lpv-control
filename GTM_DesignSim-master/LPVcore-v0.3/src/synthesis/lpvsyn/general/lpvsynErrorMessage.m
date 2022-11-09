function errorMessage = lpvsynErrorMessage(diagnostics)
%LPVSYNERRORMESSAGE Generates character array with error message 
%used in the synthesis algorithms.
%
%   Syntax:
%       errorMessage = lpvsynErrorMessage(diagnostics)
%
%   Inputs:
%       diagnostics (struct): Diagnostics information, assumed to be the
%           output of YALMIP optimize function. see <a href=
%           "matlab:help optimize">optimize</a>.
%
%   Outputs:
%       errorMessage: Character array containing the error message.
%
%   See also LPVSYN.
%

errorMessage = [sprintf('Controller synthesis failed.\n'),...
    sprintf('Solver returned error code %i: %s. ',diagnostics.problem,...
    diagnostics.info)];
end