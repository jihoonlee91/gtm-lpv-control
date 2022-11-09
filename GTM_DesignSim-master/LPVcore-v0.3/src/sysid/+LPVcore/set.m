function obj = set(obj,varargin)
% LPVCORE/SET  Modifies values of any class object.
%   This is the generic method included for all objects in the LPVcore
%   toolbox.
%      
%   OBJ = set(OBJ,'Property',VALUE) sets the property with name 'Property'
%   to the value VALUE. This is equivalent to OBJ.Property = VALUE.
% 	
%   OBJ = set(OBJ,'Property1',Value1,'Property2',Value2,...) sets multiple
%   property values in a single command.
% 	
%   OBJOUT = set(OBJ,'Property1',Value1,...) returns the modified object
%   OBJOUT.
% 	
%   See also LPVcore/GET.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
    
    n = nargin;

    if n < 2 || mod(n-1,2) ~= 0 || ~iscellstr(varargin(1:2:n-1)) %#ok<ISCLSTR>
        LPVcore.error('LPVcore:general:CompletePropertyValuePairs',[class(obj),'/set']);
    end

    
    for i = 1:(n-1)/2
        tmp = varargin{(i-1)*2+1};
        if isprop(obj, tmp)
            obj.(tmp) = varargin{i*2};
        else
            ctrlMsgUtils.error('Control:utility:pnmatch4',class(obj),tmp);
        end
        
    end
    
end