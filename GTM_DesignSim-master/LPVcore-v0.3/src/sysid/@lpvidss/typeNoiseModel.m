function Val = typeNoiseModel(obj)
%TYPENOISEMODEL Provides the type of noise model for an LPVIDSS object.
%
%   Val = typeNoiseModel(obj)
%
%   Val is either a string with 'general' or 'innovation'.
    
if obj.IsInnovationForm_
    Val = 'innovation';
else
    Val = 'general';
end

end