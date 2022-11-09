function msgstr = messageStr(msgid, varargin)
% LPVCORE.MESSAGESTR LPVcore internal utility to access translated message
% strings. MSGSTR = LPVCORE.MESSAGESTR(MSGID,PARAMS) provides the string
% associated to the message ID MSGID.
%
%   Equivalent to:
%   msg = LPVcore.message(msgid, varargin{:});
%   msgstr = msg.message;
%
%   The MESSAGE function searches only LPVcore message catalogs.
%
%   See also LPVCORE.ERROR, LPVCORE.MESSAGE, MESSAGE, CTRLMSGUTILS.ERROR

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

   try
       msg = LPVcore.message(msgid, varargin{:});
       msgstr = msg.message;
   catch Err
       throw(Err)
   end
    
end