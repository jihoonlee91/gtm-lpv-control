function bool = validatePVpairsClass(varg, className, exclList)
% LPVCORE/VALIDATEPVPAIRSCLASS  Validate Name/Property pair.
%   Helper function to validate if the property names in varg are
%   properties of the class 'className'
%      
%   bool = validatePVpairsClass(varg, className, exclList) validates if the
%   cell string varg are all properties of className excluding exclList.
%   Returns bool = true if true otherwise bool = false.
% 

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
    
    bool = true;
    
    if iscellstr(varg) || isstring(varg)

        classInfo = eval(['?',className]);
        list      = cell(length(classInfo.PropertyList),1);

        for i = 1:length(classInfo.PropertyList)

            if strcmpi(classInfo.PropertyList(i).GetAccess, 'public') && ...
                    ~any(strcmpi(classInfo.PropertyList(i).Name,exclList))

                list{i} = classInfo.PropertyList(i).Name;
            end 
        end

        list(cellfun(@isempty,list)) = []; % Remove empty elements
        
        X = cellfun(@(x)~any(strcmpi(x,list)),varg);
        if any(X); bool = find(X,1); end
        
    else
        bool = false;
    end

end





