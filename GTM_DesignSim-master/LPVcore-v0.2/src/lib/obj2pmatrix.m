function p = obj2pmatrix(obj)
%OBJ2PMATRIX Convert objects to equivalent pmatrix representation

if isa(obj, 'double')
    p = double2pmatrix(obj);
elseif isa(obj, 'pmatrix')
    p = obj;
else
    error(['Attempting to convert unsupported object of type ', ...
        class(obj), ' to pmatrix']);
end

end

