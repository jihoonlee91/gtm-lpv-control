classdef (CaseInsensitiveProperties) linfOptions
%LINFOPTIONS Create Linf options structure used for Linf based analysis and
%controller synthesis.
%
%   Syntax:
%       out = linfOptions(...)
%   
%   Optional name-value pairs:
%       'AccuracyLambda': Maximum difference between lambda search interval
%           during optimization. 
%           Default value: 1e-3.
%       'LambdaGrid': Array of values for lambda used for initial line 
%           search. 
%           Default value: linspace(1e-8, 5, 15).
%       'MaxIterations': Maximum number of iterations for optimizing 
%           lambda.
%           Defaullt value: 25;
%   
%   Outputs:
%       out: Object containing Linf options.
%
%   See also LPVSYN, LPVNORM.
%
    
    % Sorted alphabetically 
    properties (Dependent)
        AccuracyLambda
        LambdaGrid
        MaxIterations
    end

    properties (Access = protected, Hidden)
        LambdaGrid_ = linspace(1e-8, 5, 15);
        AccuracyLambda_ = 1e-3;
        MaxIterations_ = 25;
    end
    
    methods
        function obj = linfOptions(varargin)
            p = inputParser;
            names = fieldnames(obj);    % cell array with property names
            nNames = numel(names);
            
            % Add all property names of the class as parameters
            for i = 1:nNames
                addParameter(p,names{i},obj.(names{i}));
            end
                
            parse(p,varargin{:});
            
            % Get the results from the input parser and set the values
            for i = 1:nNames
                obj.(names{i}) = p.Results.(names{i});
            end
        end
        
        % LambdaGrid
        function out = get.LambdaGrid(obj)
            out = obj.LambdaGrid_;
        end

        function obj = set.LambdaGrid(obj,value)
            assert((min(size(value)) == 1) && all(value>0), ...
                ['Lambdagrid should have atleast one element ',...
                'and all elements must be greater than zero.']);
            obj.LambdaGrid_ = value;
        end

        % AccuracyLambda
        function out = get.AccuracyLambda(obj)
            out = obj.AccuracyLambda_;
        end

        function obj = set.AccuracyLambda(obj,value)
            assert(value > 0,'AccuracyLambda should be greater than 0.');
            obj.AccuracyLambda_ = value;
        end

        % MaxIterations
        function out = get.MaxIterations(obj)
            out = obj.MaxIterations_;
        end

        function obj = set.MaxIterations(obj,value)
            assert((value > 0) & (floor(value) == value),...
                'MaxIterations should be greater than 0 and integer.');
            obj.MaxIterations_ = value;
        end
    end
end

