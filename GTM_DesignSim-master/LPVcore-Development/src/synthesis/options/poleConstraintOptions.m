classdef (CaseInsensitiveProperties) poleConstraintOptions
%poleConstraintOptions Constructs matrices for closed-loop pole constraints
%use for synthesis. Multiple name-value pairs can be given for multiple
%constraints. An empty value, '[]', indicates the constraint is disabled.
%
%   Syntax:
%       out = poleConstraintOptions(...)
%   
%   Optional name-value pairs:
%       'circle': Constraint eigenvalues to |eig| < value, i.e. an open
%           circle with a radius of 'value'.
%       'leftHalfPlane': Constraint eigenvalues to Re(eig) < value, i.e.
%           a left half-plane.
%       'rightHalfPlane': Constraint eigenvalues to Re(eig) > value, i.e.
%           a right half-plane.
%       'sector': Constraint eigenvalues to Re(eig)tan(value) < -|Im(eig)|,
%           i.e. a sector in the left half complex plane with angle of
%           'value' [rad].
%       'custom': Custom pole constraint given as a structure with fields,
%           Q, S, T, and U, corresponding to the constraint:
%           [ I ]* [ Q  S ] [ I ]
%           [ z ]  [ S' R ] [ z ] < 0,
%           for all closed-loop eigenvalues 'z', where R >= 0 s.t. it is 
%           factorizable as R = T' U^(-1) T, with U > 0.
%
%   Outputs:
%       out: Structure containing Q, S, T and U, matrices corresponding to
%           the pole constraint.
%
%   See also LPVSYNOPTIONS, LPVSYN.
%
    
    % Sorted alphabetically (except for custom at the end)
    properties (Dependent)
        Circle
        LeftHalfPlane
        RightHalfPlane
        Sector
        Custom
    end

    properties (Access = protected, Hidden)
        RightHalfPlane_ = [];
        LeftHalfPlane_ = [];
        Sector_ = [];
        Circle_ = [];
        Custom_ = [];
    end

    properties (SetAccess = protected, Hidden)
        Q = [];
        S = [];
        T = [];
        U = [];
    end

    properties (Constant, Hidden)
        % Matrices corresponding to constraints
        % Naming should be '{propertyName}Mat'
        
        rightHalfPlaneMat = struct( ...
            'Q', @(value) 2*value, ...
            'S', @(value) -1, ...
            'T', @(value) 0, ...
            'U', @(value) 1 ...
            );
        leftHalfPlaneMat = struct( ...
            'Q', @(value) -2*value, ...
            'S', @(value) 1, ...
            'T', @(value) 0, ...
            'U', @(value) 1 ...
            );
        sectorMat = struct( ...
            'Q', @(value) zeros(2), ...
            'S', @(value) [sin(value),cos(value);...
                           -cos(value),sin(value)], ...
            'T', @(value) zeros(1,2), ...
            'U', @(value) 1 ...
            );
        circleMat = struct( ...
            'Q', @(value) -value^2, ...
            'S', @(value) 0, ...
            'T', @(value) 1, ...
            'U', @(value) 1 ...
            );
    end

    methods (Access = protected, Hidden)
        function obj = buildConstraints(obj)
            % Clear old matrices
            obj.Q = [];
            obj.S = [];
            obj.T = [];
            obj.U = [];
            
            % Rebuild the matrices
            names = fieldnames(obj);
            nNames = numel(names);
            
            for i = 1:nNames
                if ~isempty(obj.(names{i}))
                    obj = addConstraintMatrices(obj,names{i},obj.(names{i}));
                end
            end

            % Remove unnecessary rows from T (and corresponding rows + 
            % columns from U)
            zeroRowL = all(obj.T == 0,2);
            obj.T(zeroRowL,:)= [];
            obj.U = obj.U(~zeroRowL,~zeroRowL);
            
            % If all rows are removed, R = 0 --> R = [] (to prevent 
            % unnecessary constraint)
            if isempty(obj.U)
                obj.U = [];
                obj.T = [];
            end
        end

        function obj = addConstraintMatrices(obj,name,value)
            name = [name,'Mat'];

            obj.Q = blkdiag(obj.Q,obj.(name).Q(value));
            obj.S = blkdiag(obj.S,obj.(name).S(value));
            obj.T = blkdiag(obj.T,obj.(name).T(value));
            obj.U = blkdiag(obj.U,obj.(name).U(value));
        end
    end
    
    methods
        function obj = poleConstraintOptions(varargin)
            obj = addConstraints(obj,varargin{:});
        end

        function obj = addConstraints(obj,varargin)
            p = inputParser;
            names = fieldnames(obj); % cell array with constraint names
            nNames = numel(names);
            
            % Add all property names of the class as parameters
            for i = 1:nNames
                addParameter(p,names{i},obj.(names{i}));
            end
                
            parse(p,varargin{:});

            % Get the results from the input parser and set the values
            for i = 1:nNames
                if ~isempty(p.Results.(names{i}))
                    obj.(names{i}) = p.Results.(names{i});
                end
            end
        end

        function obj = removeConstraints(obj,varargin)
            names = fieldnames(obj); % cell array with constraint names
            removeNames = varargin;
            nRemoveNames = numel(removeNames);

            % Get the results from the input parser and set the values
            for i = 1:nRemoveNames
                index = find(strcmpi(names,removeNames{i}), 1);
                if ~isempty(index)
                    obj.(names{index}) = [];
                else
                    error([removeNames{i},' is not a valid ',...
                        'constraint name, valid names are: ',...
                        strjoin(names,', '),'.']);
                end
            end
        end

        % RightHalfPlane
        function out = get.RightHalfPlane(obj)
            out = obj.RightHalfPlane_;
        end

        function obj = set.RightHalfPlane(obj,value)
            assert(valueCheck(value), ...
                ['Value of RightHalfPlane should be a single numerical '...
                'value.']);
            obj.RightHalfPlane_ = value;

            obj = buildConstraints(obj);
        end

        % LeftHalfPlane
        function out = get.LeftHalfPlane(obj)
            out = obj.LeftHalfPlane_;
        end

        function obj = set.LeftHalfPlane(obj,value)
            assert(valueCheck(value), ...
                ['Value of LeftHalfPlane should be a single numerical '...
                'value.']);
            obj.LeftHalfPlane_ = value;

            obj = buildConstraints(obj);
        end

        % Sector
        function out = get.Sector(obj)
            out = obj.Sector_;
        end

        function obj = set.Sector(obj,value)
            assert(valueCheck(value) && value > 0, ...
                ['Value of Sector should be a single positive numerical'...
                ' value.']);
            obj.Sector_ = value;

            obj = buildConstraints(obj);
        end

        % Circle
        function out = get.Circle(obj)
            out = obj.Circle_;
        end

        function obj = set.Circle(obj,value)
            assert(valueCheck(value) && value > 0, ...
                ['Value of Circle should be a single positive numerical'...
                ' value.']);
            obj.Circle_ = value;

            obj = buildConstraints(obj);
        end

        % Custom
        function out = get.Custom(obj)
            out = obj.Custom_;
        end

        function obj = set.Custom(obj,value)
            assert(structCheck(value), ...
                ['The structure should contain a Q, S, T and U field, ',...
                 'where each field is a matrix of the correct size. '...
                 'See "help poleConstraintOptions".']);
            obj.Custom_ = value;

            obj = buildConstraints(obj);
        end
    end
end

%% Local functions
% Check value
function out = valueCheck(x)
    out = ((numel(x) == 1) && isnumeric(x)) || isempty(x);
end

% Check custom value
function out = structCheck(x)
    if ~isempty(x)
        qCheck = isnumeric(x.Q) && issymmetric(x.Q);
        sCheck = isnumeric(x.S) && (size(x.S,1) == size(x.Q,1)) && ...
            (size(x.S,2) == size(x.T,2));
        lCheck = isnumeric(x.T) && (size(x.T,1) == size(x.U,1));
        iCheck = isnumeric(x.U) && (eye(size(x.U,1)) == x.I);

    checks = (qCheck && sCheck && lCheck && iCheck);
    else
        checks = true;
    end
    out = checks || isempty(x);
end