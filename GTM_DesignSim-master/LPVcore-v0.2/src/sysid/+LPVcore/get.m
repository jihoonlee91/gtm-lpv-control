function value = get(obj,varargin)
% LPVCORE/GET  Access/query property values of any class object.
%   This is the generic method included for all objects in the LPVcore
%   toolbox.
%      
%   VALUE = get(M,'PropertyName') returns the value of the specified
%   property of the input/output model M. An equivalent syntax is 
%   VALUE = M.PropertyName.
%      
%   VALUES = get(M,{'Prop1','Prop2',...}) returns the values of several
%   properties at once in a cell array VALUES.
%         
%   get(M) displays all properties of M and their values.  
%      
%   S = get(M) returns a structure whose field names and values are the
%   property names and values of M. 
%     
%   See also LPVCORE/SET.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
 


    narginchk(1,2);     % Validate correct number of inputs
    
    n = nargin;
    
    if n == 2
    
        if ~ ( (ischar(varargin{1}) || iscellstr(varargin{1})) && isvector(varargin{1}) )
            LPVcore.error('LPVcore:general:InvalidInputArgument','second', 'character array or 1 x n cell with character arrays',class(obj));
        end

        if iscell(varargin{1})
            nv = length(varargin{1});
            value = cell(1,nv);
            for i = 1:nv
                if isprop(obj, varargin{1}{i})
                     value{i} = obj.(varargin{1}{i});
                else
                    ctrlMsgUtils.error('Control:utility:pnmatch4',class(obj),varargin{1}{i});
                end
            end
        else
            if isprop(obj, varargin{1})
                 value = obj.(varargin{1});
            else
                ctrlMsgUtils.error('Control:utility:pnmatch4',class(obj),varargin{1});
            end
        end
        
    else
        
        value = struct();
        prop = properties(obj);
        
        for i = 1:length(prop)
            value.(prop{i}) = obj.(prop{i});
        end
        
        if nargout == 0 && n == 1
            disp(value);
            clear value
        end
    end
end