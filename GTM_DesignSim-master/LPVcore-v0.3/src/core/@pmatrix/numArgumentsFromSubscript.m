function n = numArgumentsFromSubscript(obj,varargin)
    %SUBSREF and SUBSASGN calls this first if it is overloaded
    %instead of numel.
    
    %this function needs to be in a seperate file to work TODO: find out why 
    n = builtin('numel', obj);
end