function error( errId, varargin )
% LPVCORE.ERROR(errId, varargin) goes from errId to error message then error
%   Will translate a messageId into a string and pass both of them to
%   the MATLAB error function.
%
%   See also LPVCORE.MESSAGE, MESSAGE, CTRLMSGUTILS.ERROR

%   To use this function for an already created error id
%   call LPVcore.error( errId, args)
%
%   This function is closely related to ctrlMsgUtils.error( errId, args).
%   However, custom toolboxes cannot uses the message catalog in MATLAB,
%   e.g., 'message' function
%
%   Valid syntax for errId in DAStudio.error is
%
%   LPVcore:component:messageId
%
%   The variable arguments args are used to fill in the predefined 
%   holes in the message string. The XML catalog message library can be
%   found in resources/en/

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

msg = LPVcore.message(errId, varargin{:});

% Throw error as caller
throwAsCaller(MException(errId, '%s', msg.message))

