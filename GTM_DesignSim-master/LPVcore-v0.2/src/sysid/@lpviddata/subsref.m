function varargout = subsref(obj,s,varargin)
%SUBSREF Construct a subselection of the data set (lpviddata object).
%
%   This function will overload the buildin subsref function of Matlab.
%
%   data = lpviddata(y,p,u,Ts);
%
%   tmp = data(idxT); tmp is a subselection of the data set containing the
%   data samples of the output, scheduling, and input signal at time
%   indexes idxT.
%
%   tmp = data(idxT, idxY, idxP, idxU); tmp is a subselection of the data
%   set containing the data samples of the idxY-th output chanels, idxP-th
%   scheduling chanels, and idxU-th input channels at time indexes idxT.
%
%   Examples
%
%   tmp = data(1:20); tmp is a subselection of data containing the first 21
%   time samples of the output, scheduling, and input signal.
%
%   tmp = data(20:end); tmp is a subselection of data containing the last
%   end-20 time samples of the output, scheduling, and input signal.
%
%   tmp = data(20:end, [2 3], :, 4);
%
%   See also LPVIDDATA


% https://nl.mathworks.com/help/matlab/matlab_oop/code-patterns-for-subsref-and-subsasgn-methods.html
% To include 'end' command, we require the end function of the lpviddata
% object.

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).


    switch s(1).type
        case '.'
            try [varargout{1:nargout}] = builtin('subsref',obj,s);
            catch ME; throw(ME); end
            
        case '()'
            
            if length(s) == 1
             
                exclList = {'InputData','SchedulingData','OutputData','Ts','SamplingInstants','Domain','InterSample'};

                if length(s.subs) == 1
                    
                    if ischar(s.subs{1}) && strcmp(s.subs{1},':')
                        varargout{1} = obj;
                        return;                       
                    end
                    
                    try
                        if ~isempty(obj.y); y = obj.y(s.subs{1},:);
                        else; y = [];end

                        if ~isempty(obj.u); u = obj.u(s.subs{1},:);
                        else; u = [];end

                        propList     = localCopyProps(obj,exclList);
                        varargout{1} = lpviddata(y,obj.p(s.subs{1},:),u,obj.Ts,propList{:});
                        
                    catch ME
                        throw(ME)
                    end

                elseif length(s.subs) == 4
                    
                    try
                        if ~isempty(obj.y) && ~isempty(s.subs{2}); y = obj.y(s.subs{1},s.subs{2});
                        else; y = [];end
                        
                        if isempty(s.subs{3})
                            LPVcore.error('LPVcore:general:InvalidInputArgument','third','nonempty vector with indeces. The lpviddata object requires a scheduling signal. Otherwise, use "iddata"','lpviddata');
                        end

                        if ~isempty(obj.u) && ~isempty(s.subs{4}); u = obj.u(s.subs{1},s.subs{4});
                        else; u = [];end

                        propList     = localCopyProps(obj,exclList);
                        
                        % Modify InputName, InputUnit
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'InputName')), propList(1:2:end-1)),1);
                        if ~isempty(propList{idx*2}); propList{idx*2} = propList{idx*2}(s.subs{4}); end
                        
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'InputUnit')), propList(1:2:end-1)),1);
                        if ~isempty(propList{idx*2}); propList{idx*2} = propList{idx*2}(s.subs{4}); end
                        
                        % Modify SchedulingName, SchedulingUnit
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'SchedulingName')), propList(1:2:end-1)),1);
                        propList{idx*2} = propList{idx*2}(s.subs{3}); % Always defined
                        
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'SchedulingUnit')), propList(1:2:end-1)),1);
                        if ~isempty(propList{idx*2}); propList{idx*2} = propList{idx*2}(s.subs{3}); end
                        
                        % Modify OutputName, OutputUnit
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'OutputName')), propList(1:2:end-1)),1);
                        if ~isempty(propList{idx*2}); propList{idx*2} = propList{idx*2}(s.subs{2}); end
                        
                        idx = find(cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,'OutputUnit')), propList(1:2:end-1)),1);
                        if ~isempty(propList{idx*2}); propList{idx*2} = propList{idx*2}(s.subs{2}); end
                        
                        varargout{1} = lpviddata(y,obj.p(s.subs{1},s.subs{3}),u,obj.Ts,propList{:});
                        
                    catch ME
                        throw(ME)
                    end
                   
                else
                    ctrlMsgUtils.error('MATLAB:badsubscript','Subscript indices must either be real positive integers or logicals.');
                end
                
                
                
            else
                [varargout{1:nargout}] = builtin('subsref',obj,s);
            end
            
            
        otherwise
            ctrlMsgUtils.error('MATLAB:cellRefFromNonCell');
    end

end


%-------------------- LOCAL FUNCTIONS -------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function propList = localCopyProps(obj,exclList)
    % Create a cell with all properties and its values of obj. This can be
    % used to create new object and assign all props. The properties in
    % exclList will not be taken into account.
    
    prop = properties(obj);
    n = length(prop);
    propList = cell(n*2,1);
    propList(1:2:end-1) = prop;

    for i = 1:n
        propList{i*2} = obj.(prop{i});
    end

    idx = cellfun(@(x) ischar(x) && isvector(x) && any(strcmp(x,exclList)), propList(1:2:end-1));
    
    if any(idx)
        idx = repmat(idx',[2 1]);
        idx = idx(:);
        propList(idx) = [];
    end
end