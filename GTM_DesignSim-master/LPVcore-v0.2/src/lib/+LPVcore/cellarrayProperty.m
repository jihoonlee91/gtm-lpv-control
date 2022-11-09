function val = cellarrayProperty(name, val, sz, default)
    % Convert text-based cell array property using MATLAB style conventions
    % for e.g., InputName property of SS object.
    %
    %   Syntax:
    %       s = cellarrayProperty(name, s, sz, default)
    %
    %   Inputs:
    %       name: name of the property
    %       s: cell array of sz-by-1 or 1-by-sz, or a character vector
    %       sz: integer denoting the number of elements the array must have
    %       default: character vector denoting the default text (which gets
    %           expanded in case sz > 1.
    %
    %   Outputs:
    %       s: cell array 
    %
    
    if nargin <= 3
        default = '';
    end
    
    % Convert to cell array
    if ~iscell(val); val = {val}; end
    
    % Populate with default if empty
    if isempty(val) && sz > 0; val = {default}; end
    
    % Transpose from row to column if needed
    if isrow(val); val = val.'; end
    
    % Check if cell array was passed which was neither row nor column
    if sz > 0 && ~iscolumn(val)
        error([name, ' must be a row or column cell array.']);
    end
    
    % Check if all elements of cell are character vectors
    if ~iscellof(val, 'char')
        error([name, ' must be a cell array of character vectors.']);
    end
    
    % Expand scalar if required
    if sz > 1 && numel(val) == 1
        tmp = val{1};
        % '' should not be expanded to {'(1)', ...}
        if ~isempty(tmp)
            for i=1:sz
                val{i, 1} = [tmp, '(', int2str(i), ')'];
            end
        else
            val{sz, 1} = '';
        end
    end
    
    % Check size constraints
    if numel(val) ~= sz
        error([name, ' must be a cell array of character vectors ', ...
            'with ', int2str(sz), ' elements.']);
    end
end
