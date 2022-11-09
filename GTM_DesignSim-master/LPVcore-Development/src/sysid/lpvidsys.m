classdef (Abstract, Hidden) lpvidsys
    %LPVIDSYS Parent class for systems with identifiable parameters
    
    properties (Abstract)
        Type  % Character vector denoting the type of system, e.g. "ARX" or "SS"
    end
    
    properties
        % Structure containing identification report with the following
        % fields:
        %
        %   Status: character vector denoting how the model was created or
        %       estimated.
        %
        %   Fit: structure containing prediction fit and simulation fit.
        %
        %   Method: method used for the identification.
        %
        %   OptionsUsed: option set used for the command under 'Method'.
        %
        %   DataUsed: lpviddata object used for the command under 'Method'.
        %
        Report = struct(...
            'Status', 'Created by direct construction or transformation. Not estimated.', ...
            'Method', '', ...
            'Fit', struct('PredictionFit', [], ...
                'SimulationFit', []), ...
            'OptionsUsed', [], ...
            'DataUsed', []);
    end
   
end

