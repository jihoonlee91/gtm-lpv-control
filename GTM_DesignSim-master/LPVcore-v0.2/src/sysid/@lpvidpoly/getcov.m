function V = getcov(obj)
%GETCOV Get parameter covariance information in either raw
%or factored form.
%
%  VALUE = GETCOV(SYS) returns the data on covariance of
%     parameters of identified model SYS.
%
% See also NPARAMS, GETCOV

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    try
       V = getValue(obj.Covariance);
    catch E
       throw(E)
    end

end