function index = end(X,position,numindices)
%END (overloaded)

switch position
    case {1,2}       
        if numindices == 1
            % User has written someting like X(end) or X(1:end)
            index = numel(X);
        else
            index = size(X,position);
        end
    otherwise
        index = 1;
end
