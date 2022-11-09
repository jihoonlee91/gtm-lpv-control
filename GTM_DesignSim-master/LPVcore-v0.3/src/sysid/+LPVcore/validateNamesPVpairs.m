function bool = validateNamesPVpairs(varg, list)
% LPVCORE/VALIDATENAMESPVPAIRS  Validates Name/Property pairs for LPVcore
%   classes. Helper function to validate if the property names in varg are
%   in the list.
%      
%   bool = validateNamesPVpairs(varg, list) validates if the cell string
%   varg are all contained in the cell string list. Returns bool = true if
%   true otherwise bool = false.
% 

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
    
    bool = true;
    
    if iscellstr(varg) || isstring(varg)
        X = cellfun(@(x)strcmpi(x,list),varg,'UniformOutput',false);
        X = vertcat(X{:});
        if any(~any(X,2)); bool = find(~any(X,2),1); end
        
    else
        bool = false;
    end

end