function varargout = subsref(obj, s)
%SUBSREF Overloaded for gridvar objects.
%

% Template from: https://nl.mathworks.com/help/matlab/matlab_oop/code-patterns-for-subsref-and-subsasgn-methods.html
switch s(1).type
    case '.'
        if length(s) == 2 && strcmp(s(2).type,'{}')
            % Implement obj.PropertyName{indices}
            [varargout{1:nargout}] = builtin('subsref',obj.matrices,s(2));
        else
            % Use built-in for any other expression
            [varargout{1:nargout}] = builtin('subsref',obj,s);
        end
    case '()'
        if length(s) == 1
            % Implement obj(indices)
            A = obj;
            A.matrices = cellfun(@(mat) subsref(mat,s),obj.matrices,'UniformOutput',false);
            varargout{1} = A;
        elseif length(s) == 2 && strcmp(s(2).type,'.')
            % Implement obj(ind).PropertyName
            A = obj;
            A.matrices = cellfun(@(mat) subsref(mat,s(1)),obj.matrices,'UniformOutput',false);
            [varargout{1:nargout}] = builtin('subsref',A,s(2));
        else
            % Use built-in for any other expression
            [varargout{1:nargout}] = builtin('subsref',obj,s);
        end
    otherwise
        error('Not a valid indexing expression')
end