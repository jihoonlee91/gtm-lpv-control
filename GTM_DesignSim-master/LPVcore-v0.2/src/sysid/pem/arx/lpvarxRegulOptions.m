function obj = lpvarxRegulOptions(varargin)
%LPVARXREULOPTIONS  Creates option set for the LPVARXREGUL command.
%
%   OPT = arxRegulOptions returns the default options for ARX.
%
%   OPT = arxRegulOptions('Option1',Value1,'Option2',Value2,...) uses
%   name/value pairs to override the default values for
%   'Option1','Option2',...
%
%   The supported options are:
%   
%   InputOffset   Specifies the offset levels present in the input signals
%                 of the estimation data. Specify as an Nu-element column
%                 vector, where Nu is the number of inputs. Default is []
%                 which implies no offsets.
%
%   SchedulingOffset Scheduling signal offset levels. Similar
%                 interpretation as InputOffset. The specified values are
%                 subtracted from the scheduling signals before they are
%                 used for estimation.
%
%   OutputOffset  Output signal offset levels. Similar interpretation as
%                 InputOffset. The specified values are subtracted from the
%                 output signals before they are used for estimation.
%
%   OutputWeight  Weighting of prediction errors in multioutput
%                 estimations (irrelevant for single output estimations).
%                 Specify as a positive semi-definite matrix of size equal
%                 to the number of outputs. This amounts to minimizing
%                 trace(E'*E*W/N), where W is the specified matrix, E is
%                 the prediction error matrix and N is the number of
%                 samples. Default: [] implying Identity matrix for W.
%
%   Regularization Specify regularization scaling Lambda and weighting
%                 matrix R. Lambda must be zero (no regularization;
%                 default) or a positive scalar. R must be a positive
%                 semidefinite matrix of size equal to the number of model's
%                 free parameters (np = sum(sum([na nb]) for ARX models).
%                 Use the "arxRegul" function to generate suitable values
%                 for Lambda and R.
%
%
%   See also LPVARX, ARXREGUL.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

try
   obj = initOptions(LPVcore.idoptions.lpvarxRegul,varargin);
catch E
   throw(E);
end

end