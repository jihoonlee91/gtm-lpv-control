function idx = end(obj,k, ~)
%END The end function of the lpviddata object.
%
%   ind = end(obj,k,n)
%
%   The arguments are described as follows:
% 
%       k is the index in the expression using the end syntax
%       n is the total number of indices in the expression
% 
%       ind is the index value to use in the expression
%
%   See also LPVIDDATA/SUBSREF, LPVIDDATA

% https://nl.mathworks.com/help/matlab/matlab_oop/object-end-indexing.html

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).

    if k > 4
        idx = 1;
    elseif k == 1
        idx = obj.lengthDataSet;
    else
        idx = obj.OSISize(k-1);
    end

end