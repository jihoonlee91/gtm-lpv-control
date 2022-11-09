classdef (CaseInsensitiveProperties) lpvsynOptions
%LPVSYNOPTIONS Create LPV synthesis options object.
%
%   Syntax:
%       out = lpvsynOptions(...)
%   
%   Optional name-value pairs:
%       'Backoff': Correction factor for gamma, used in the synthesis to 
%           reduce numerical errors, since the true optimal solution is
%           close to singular. 
%           Default value: 1.01.
%       'Dependency': Array (1x4 or 4x1) indicating which controller
%           matrices, corresponding to [A,B,C,D], should have parameter-
%           dependency. In case of the transformation method, these apply
%           to the transformed controller matrices.
%           Default value: [1 1 1 1].
%       'DirectFeedthrough': Option indicating if controller has direct
%           feedthrough (1) or not (0).
%           Default value: 1;
%       'ImproveConditioning': Solves extra optimization problems to
%           improve the conditioning of found solutions.
%           Default value: 1;
%       'LinfOptions': LinfOptions object containing the options for Linf 
%           based synthesis. See <a href="matlab:helpPopup linfOptions"
%           >linfOptions</a>.
%           Default value: linfOptions().
%       'Method': Indicates what method to use for synthesis. Possible
%           values: 'transformation', 'projection'. 
%           Default value: 'transformation'.
%       'NumericalAccuracy': Numerical accuracy parameter. Used for solving
%           the LMIs (LMI <= -NumAcc). 
%           Default value: 1e-8.
%       'ParameterVaryingStorage': Indicates if a parameter-varying (1) or
%           parameter independent (0) storage function is considered. If
%           set to [], lpvsyn will check if an parameter-varying storage
%           function can be used or not.
%           Default value: [].
%       'Performance': Perfomance metric that is considered for synthesis.
%           Possible values: 'linf', 'l2', 'h2', 'passive'. 
%           Default value: 'l2'.
%       'PoleConstraint': PoleConstraintOptions object describing the
%           constraints on the closed-loop eigenvalues, see
%           <a href="matlab:helpPopup poleConstraintOptions"
%           >poleConstraintOptions</a>.
%           Default value: poleConstraintOptions().
%       'SolverOptions': Structure containing the YALMIP solver settings. 
%           See <a href="matlab:helpPopup sdpsettings">sdpsettings</a>. 
%           Default value: sdpsettings('verbose',0,'solver','sdpt3').
%       'Verbose': Turns progress messages on (1) or off (0);
%           Default value: 1;
%       
%   Outputs:
%       out: Object containing LPV synthesis options.
%   
%   See also LPVSYN.
%
    
    % Sorted alphabetically 
    properties (Dependent)
        Backoff
        Dependency
        DirectFeedthrough
        ImproveConditioning
        LinfOptions
        Method
        NumericalAccuracy
        ParameterVaryingStorage
        Performance
        PoleConstraint
        SolverOptions
        Verbose
    end

    % Properties set and used in lpvsyn
    properties (Hidden)
        MethodChangedFlag = false
        SystemType
    end

    properties (Access = protected, Hidden)
        Performance_ = 'l2';
        Backoff_ = 1.01;
        SolverOptions_ = sdpsettings('verbose',0,'solver','sdpt3',...
            'cachesolvers',1);
        NumericalAccuracy_ = 1e-8;
        Method_ = 'transformation';
        ParameterVaryingStorage_ = [];
        LinfOptions_ = linfOptions;
        Dependency_ = logical([1 1 1 1]);
        Verbose_ = true;
        DirectFeedthrough_ = true;
        ImproveConditioning_ = true;
        PoleConstraint_ = poleConstraintOptions;
    end

    properties (Constant, Hidden)
        possiblePerformance = {'linf','l2','h2','passive'};
        possibleMethod = {'transformation','projection'};
        possibleSystemType = {'affine','grid'};
    end
    
    methods
        function obj = lpvsynOptions(varargin)
            p = inputParser;
            names = fieldnames(obj);    % cell array with property names
            nNames = numel(names);
            
            % Add all property names of the class as parameters
            for i = 1:nNames
                addParameter(p,names{i},obj.(names{i}));
            end
                
            parse(p,varargin{:});

            % Names of options that don't use default value, i.e. that have
            % been set by the user
            setNames = setxor(p.UsingDefaults,p.Parameters);
            nSetNames = numel(setNames);
            
            % Set the values
            for i = 1:nSetNames
                obj.(setNames{i}) = p.Results.(setNames{i});
            end
        end
        
        % Performance
        function out = get.Performance(obj)
            out = obj.Performance_;
        end

        function obj = set.Performance(obj,value)
            value = lower(value);
            assert(any(strcmp(value,obj.possiblePerformance)), ...
                ['Possible performance metrics are: ', ...
                strjoin(obj.possiblePerformance,', '),'.']);
            obj.Performance_ = value;
        end

        % Backoff
        function out = get.Backoff(obj)
            out = obj.Backoff_;
        end

        function obj = set.Backoff(obj,value)
            assert(value >= 1,'Backoff should be greater than 1.');
            obj.Backoff_ = value;
        end

        % SolverOptions
        function out = get.SolverOptions(obj)
            out = obj.SolverOptions_;
        end

        function obj = set.SolverOptions(obj,value)
            assert(isa(value,"struct"),...
                ['SolverOptions should be sdpsettings structure, ', ...
                 'see "help sdpsettings".']);
            value.cachesolvers = 1;     % force cache solvers to be 1 for
                                        % small speed up
            obj.SolverOptions_ = value;
        end

        % NumericalAccuracy
        function out = get.NumericalAccuracy(obj)
            out = obj.NumericalAccuracy_;
        end

        function obj = set.NumericalAccuracy(obj,value)
            assert(value > 0,['NumericalAccuracy should be greater ',...
                'than 0.']);
            obj.NumericalAccuracy_ = value;
        end

        % Method
        function out = get.Method(obj)
            out = obj.Method_;
        end

        function obj = set.Method(obj,value)
            value = lower(value);
            assert(any(strcmp(value,obj.possibleMethod)), ...
                ['Possible methods are: ', ...
                strjoin(obj.possibleMethod,', '),'.']);
            obj.Method_ = value;
            obj.MethodChangedFlag = true;
        end

        % ParameterVaryingStorage
        function out = get.ParameterVaryingStorage(obj)
            out = obj.ParameterVaryingStorage_;
        end

        function obj = set.ParameterVaryingStorage(obj,value)
            obj.ParameterVaryingStorage_ = logical(value);
        end

        % LinfOptions
        function out = get.LinfOptions(obj)
            out = obj.LinfOptions_;
        end

        function obj = set.LinfOptions(obj,value)
            assert(isa(value,"linfOptions"),...
                ['LinfOptions should be an linfOptions object, ',...
                 'see "help linfOptions".']);
            obj.LinfOptions_ = value;
        end

        % Dependency
        function out = get.Dependency(obj)
            out = obj.Dependency_;
        end

        function obj = set.Dependency(obj,value)
            assert(all(size(reshape(value,1,[])) == [1,4]),...
                'Dependency should be an array with 4 elements.');
            obj.Dependency_ = logical(value);
        end

        % Verbose
        function out = get.Verbose(obj)
            out = obj.Verbose_;
        end

        function obj = set.Verbose(obj,value)
            obj.Verbose_ = logical(value);
        end

        % DirectFeedthrough
        function out = get.DirectFeedthrough(obj)
            out = obj.DirectFeedthrough_;
        end

        function obj = set.DirectFeedthrough(obj,value)
            obj.DirectFeedthrough_ = logical(value);
        end

        % ImproveConditioning
        function out = get.ImproveConditioning(obj)
            out = obj.ImproveConditioning_;
        end

        function obj = set.ImproveConditioning(obj,value)
            obj.ImproveConditioning_ = logical(value);
        end

        % PoleConstraint
        function out = get.PoleConstraint(obj)
            out = obj.PoleConstraint_;
        end

        function obj = set.PoleConstraint(obj,value)
            assert(isa(value,"poleConstraintOptions"),...
                ['PoleConstraint should be an poleConstraintOptions ',...
                'object, see "help poleConstraintOptions".']);
            obj.PoleConstraint_ = value;
        end

        % SystemType
        function out = get.SystemType(obj)
            out = obj.SystemType;
        end

        function obj = set.SystemType(obj,value)
            value = lower(value);
            assert(any(strcmp(value,obj.possibleSystemType)), ...
                ['Possible systemTypes are: ', ...
                strjoin(obj.possibleSystemType,', '),'.']);
            obj.SystemType = value;
        end
    end
end

