classdef lpvss < pmodel.Generic
% Parameterization of linear parameter-varying state-space models.
%
%   The LPVCORE.PMODEL.LPVSS class parameterizes state-space models of
%   the form
%       dx(t)/dt = A(rho(t)) x(t) + B(rho(t)) u(t) + K(rho(t)) e(t)
%         y(t)   = C(rho(t)) x(t) + D(rho(t)) u(t) + e(t)
%
%    or
%
%       dx(t)/dt = A(rho(t)) x(t) + B(rho(t)) u(t) + G(rho(t)) v(t)
%         y(t)   = C(rho(t)) x(t) + D(rho(t)) u(t) + H(rho(t)) e(t)
%
%   M = pmodel.ss(A,B,C,D,K) returns a parametric state-space model M where
%   the model parameters are initialized to the values A,B,C,D,K.
%
%   M = pmodel.ss(A,B,C,D,G,H) returns a parametric state-space model M
%   where the model parameters are initialized to the values A,B,C,D,G,H.
%
%   M = pmodel.ss(A,B,C,D) constructs a parametric state-space model M
%   without noise input e, (w,v). This is equivalent to setting M.K, M.G,
%   and M.H to empty.
%
%   We deviate from the LTI implementation pmodel.polynomial, due to the
%   LPV nature. Our polynomials are strucutres with:
%
%   A.Parameter(j) = param.Continuous('A',eye(ny));
%   A.Basis         = {@(x) x(:,1), ... };
%
%   e.g. A.Parameter(1) = param.Continuous('A',eye(ny));
%        A.Parameter(2) = param.Continuous('A',eye(ny));
%        A.Basis        = {@(x) x(:,1), @(x) x(:,4) };
%
%   To get the pmatrix as an ouput, use
%   LPVcore.pmodel.lpvss.getPmatix(Prop) with Prop = {'a', 'b', 'c', 'd',
%   'k', 'g', 'h'}.
%
%   See also PMODEL.SS, LPVCORE.PMODEL.LPVPOLYNOMIAL

% 10/11/2018
%
% (c) Copyright 2018 - Control systems group, University of Eindhoven
% dr. Roland Toth (r.toth@tue.nl).
% 
% Original developer of the script: dr. Pepijn B. Cox (p.b.cox@tue.nl).
%
% Note: template is pmodel.polynomial. 

   properties
      % A matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of the state matrix A,
      % to initialize A, or to fix/free specific entries of A.
      A = []
      % B matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of the input-to-state
      % matrix B, to initialize B, or to fix/free specific entries of B.
      B = []
      % C matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of the state-to-output
      % matrix C, to initialize C, or to fix/free specific entries of C.
      C = []
      % D matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of the feedthrough
      % matrix D, to initialize D, or to fix/free specific entries of D.
      % For example, you can fix D to zero by typing
      %    M.D.Value = 0;  M.D.Free = false;
      D = []
      % K matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of K, initialize K,
      % or to fix/free specific entries of K. You can also set K to []
      % to eliminate the noise input e.
      K = []
      % G matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of G, initialize G,
      % or to fix/free specific entries of G. You can also set G to []
      % to eliminate the noise input v.
      G = []
      % H matrix (matrix-valued parameter).
      %
      % Use this property to read the current value of H, initialize H,
      % or to fix/free specific entries of H. You can also set H to []
      % to eliminate the noise input e.
      H = []
      % X0 vector.
      %
      % Initial value of the state. If it is set to [], then it is not
      % considered to be estimated.
      X0 = []
      % Covariance of X0.
      %
      % See idpack.FactoredCovariance()
      X0CovT = idpack.FactoredCovariance; % Note that cov = inv(X0CovT.R'*X0CovT.R);  This is executed when cov = getValue(X0CovT);
      % State names (string vector, default = 0x1 cell string vector).
      %
      % You can set this property to:
      %   * A cell string for first-order models, for example, {'position'}
      %   * A cell string vector for models with two or more states, for example,
      %     {'position' ; 'velocity'}
      % Use the empty string '' for unnamed states.
      StateName = cell(0,1);
      % State units (string vector, default = 0x1 cell string vector).
      %
      % Use this property to keep track of the units each state is expressed in.
      % You can set "StateUnit" to:
      %   * A cell string for first-order models, for example, {'m/s'}
      %   * A cell string vector for models with two or more states, for example,
      %    {'m' ; 'm/s'}
      StateUnit = cell(0,1);
      % SchedulingTimeMap - The scheduling time-map associated to all
      % basis functions.
      % See also TIMEMAP.
      %
      % Default: []
      SchedulingTimeMap = []
   end
   
   properties (Access=private)
       AtypeBasis = {};
       BtypeBasis = {};
       CtypeBasis = {};
       DtypeBasis = {};
       KtypeBasis = {};
       GtypeBasis = {};
       HtypeBasis = {};
   end
   
   
   methods (Access = protected)
      
      % PARAMETER SERIALIZATION INTERFACE (PMODEL.GENERIC)
      function ps = getParamSet(M)
          if isempty(M.A) || isempty(M.A.Parameter(1).Value)
              psA = [];
          else
              psA = [];
              for i = 1:numel(M.A.Parameter);  psA = cat(1,psA,M.A.Parameter(i)); end
          end
          
          if isempty(M.B) || isempty(M.B.Parameter(1).Value)
              psB = [];
          else
              psB = [];
              for i = 1:numel(M.B.Parameter); psB = cat(1,psB,M.B.Parameter(i)); end
          end
          
          if isempty(M.C) || isempty(M.C.Parameter(1).Value)
              psC = [];
          else
              psC = [];
              for i = 1:numel(M.C.Parameter); psC = cat(1,psC,M.C.Parameter(i)); end
          end
          
          if isempty(M.D) || isempty(M.D.Parameter(1).Value)
              psD = [];
          else
              psD = [];
              for i = 1:numel(M.D.Parameter); psD = cat(1,psD,M.D.Parameter(i)); end
          end
          
          if isempty(M.K) || isempty(M.K.Parameter(1).Value)
              psK = [];
          else
              psK = [];
              for i = 1:numel(M.K.Parameter); psK = cat(1,psK,M.K.Parameter(i)); end
          end
          
          if isempty(M.G) || isempty(M.G.Parameter(1).Value)
              psG = [];
          else
              psG = [];
              for i = 1:numel(M.G.Parameter); psG = cat(1,psG,M.G.Parameter(i)); end
          end
          
          if isempty(M.H) || isempty(M.H.Parameter(1).Value)
              psH = [];
          else
              psH = [];
              for i = 1:numel(M.H.Parameter)
                  psH = cat(1,psH,M.H.Parameter(i));
              end
          end
          
          ps = [psA; psB; psC; psD; psK; psG; psH];
      end
      
      function M = setParamSet(M,ps)
          psN = {ps.Name};
          
          idA = find(strcmp(psN,'A'));  idB = find(strcmp(psN,'B'));
          idC = find(strcmp(psN,'C'));  idD = find(strcmp(psN,'D'));
          idK = find(strcmp(psN,'K'));  idG = find(strcmp(psN,'G'));
          idH = find(strcmp(psN,'H'));
          
          for i = 1:numel(idA); M.A.Parameter(i) = ps(idA(i)); end
          for i = 1:numel(idB); M.B.Parameter(i) = ps(idB(i)); end
          for i = 1:numel(idC); M.C.Parameter(i) = ps(idC(i)); end
          for i = 1:numel(idD); M.D.Parameter(i) = ps(idD(i)); end
          for i = 1:numel(idK); M.K.Parameter(i) = ps(idK(i)); end
          for i = 1:numel(idG); M.G.Parameter(i) = ps(idG(i)); end
          for i = 1:numel(idH); M.H.Parameter(i) = ps(idH(i)); end
          
      end
      
      function dispScalarModel(M)
         % Display of a scalar ss object.
         S = struct('A',M.A,'B',M.B,'C',M.C,'D',M.D);
         if ~isequal(M.K,[])
            S.K = M.K;
         end
         if ~isequal(M.G,[])
            S.G = M.G;
         end
         if ~isequal(M.H,[])
            S.H = M.H;
         end
         S.StateName = M.StateName;
         S.StateUnit = M.StateUnit;
         disp(S)
      end
      
      function dispModelType(~,~)
         % Display model type information.
         disp(LPVcore.messageStr('LPVcore:lpvidss:lpvssmodel'));
      end
   end
   
   % PUBLIC METHODS
   methods
       function M = lpvss(a,b,c,d,k,h)
           % Constructs LPVcore.pmodel.lpvss instance. This object does not
           % include validation of each set method. This because we assume
           % that LPVcore.pmodel.lpvss is protected by other classes, such as
           % lpvss.
           ni = nargin;
           if ni==0
               return
           end
           
           M = setABCDKGH(M, 'A', a);
           M = setABCDKGH(M, 'B', b);
           M = setABCDKGH(M, 'C', c);
           M = setABCDKGH(M, 'D', d);
           
           if ni==5
               M = setABCDKGH(M, 'K', k);
           elseif ni>5
               M = setABCDKGH(M, 'G', k);
               M = setABCDKGH(M, 'H', h);
           end
           
       end
      
%       function M = scaleTime(M,sf,Ts)
%          % Time vector scaling tnew = sf * told
%       end

        function mat = getPmatrix(M,Prop)
            
            if isempty( M.([Prop,'typeBasis']) )
                mat = [];
            elseif isequal( M.([Prop,'typeBasis']), {0,[]} )
                mat = M.(Prop).Parameter.Value;
            else
                npsi = numel(M.(Prop).Parameter);
                tmpMatrices = zeros( [ size(M.(Prop).Parameter(1).Value), npsi] );
                for i = 1:npsi
                     tmpMatrices(:,:,i) = M.(Prop).Parameter(i).Value;
                end
                
                mat = pmatrix.constructPmatrix(tmpMatrices, M.(Prop).Basis, ... 
                                M.SchedulingTimeMap, M.([Prop,'typeBasis']));
            end
        end

   end  % end of public methods
   
   methods (Hidden)
      
      function nx = order(M)
         % Number of states
         
         nx = 0;
         if ~isempty(M.A) && ~isempty(M.A.Parameter(1).Value); nx = size(M.A.Parameter(1).Value,1); return; end
         if ~isempty(M.B) && ~isempty(M.B.Parameter(1).Value); nx = size(M.B.Parameter(1).Value,1); return; end
         if ~isempty(M.K) && ~isempty(M.K.Parameter(1).Value); nx = size(M.K.Parameter(1).Value,1); return; end
         if ~isempty(M.G) && ~isempty(M.G.Parameter(1).Value); nx = size(M.G.Parameter(1).Value,1); return; end
      end
      
      function s = iosize(M,dim)
         % I/O sizes
         
         if isempty(M.D) || isempty(M.D.Parameter(1).Value)
             
             s = [0 0];
             % Find ny
             if ~isempty(M.C) && ~isempty(M.C.Parameter(1).Value); s(1) = size(M.C.Parameter(1).Value,1); end
             if ~isempty(M.K) && ~isempty(M.K.Parameter(1).Value); s(1) = size(M.K.Parameter(1).Value,2); end
             if ~isempty(M.H) && ~isempty(M.H.Parameter(1).Value); s(1) = size(M.H.Parameter(1).Value,1); end
             
             % Find nu
             if ~isempty(M.B) && ~isempty(M.B.Parameter(1).Value); s(2) = size(M.B.Parameter(1).Value,2); end             
         else
             s = getSize(M.D.Parameter(1));
         end
         
         if nargin>1
             if dim<3; s = s(dim);
             else;     s = 1;      end
         end
         
      end
      
%       function M1 = iocat(dim, M1, M2)
%          % Concatenate models along input (2) or output (1) dimension.
%       end
      
%       %-------------------------------------------
%       function M = getsubsys(M,rowIndex,colIndex)
%       end
      
      %-------------------------------------------------------------------
      function M = checkDataSize(M)
         % Checks size consistency for A,B,C,D,K or A,B,C,D,G,H parameters.
         % First run checkDataType before checkDataSize
         
         Aisemp = isempty(M.A) || isempty(M.A.Parameter(1).Value);
         Bisemp = isempty(M.B) || isempty(M.B.Parameter(1).Value);
         Cisemp = isempty(M.C) || isempty(M.C.Parameter(1).Value);
         Disemp = isempty(M.D) || isempty(M.D.Parameter(1).Value);
         Kisemp = isempty(M.K) || isempty(M.K.Parameter(1).Value);
         Gisemp = isempty(M.G) || isempty(M.G.Parameter(1).Value);
         Hisemp = isempty(M.H) || isempty(M.H.Parameter(1).Value);
         
         if Aisemp; nx = 0;
         else;      nx = size(M.A.Parameter(1).Value,1); end
         
         if Disemp
             ny = 0; nu = 0;
             if ~Cisemp; ny = size(M.C.Parameter(1).Value,1); end
             
             if ~Bisemp;  nu = size(M.B.Parameter(1),2); end
         else
             [ny, nu] = size(M.D.Parameter(1).Value);
         end
         
         if ~Cisemp && (Aisemp && Bisemp && Kisemp && Gisemp )
            LPVcore.error('LPVcore:general:InvalidInput','"A", "B", "K", "G" property cannot be empty and "C" nonempty.','LPVcore.pmodel.lpvss');
         end
         
         if ~Kisemp && (~Gisemp || ~Hisemp )
            LPVcore.error('LPVcore:lpvidss:notfilledKGH','LPVcore.pmodel.lpvss');
         end
         
         try
             if ~Aisemp; localCheckDataSizeProp(M.A, M.AtypeBasis, [nx nx], 'A'); end
             if ~Bisemp; localCheckDataSizeProp(M.B, M.BtypeBasis, [nx nu], 'B'); end
             if ~Cisemp; localCheckDataSizeProp(M.C, M.CtypeBasis, [ny nx], 'C'); end
             if ~Disemp; localCheckDataSizeProp(M.D, M.DtypeBasis, [ny nu], 'D'); end
             if ~Kisemp; localCheckDataSizeProp(M.K, M.KtypeBasis, [nx ny], 'K'); end
             if ~Gisemp; localCheckDataSizeProp(M.G, M.GtypeBasis, [nx nx], 'G'); end
             if ~Hisemp; localCheckDataSizeProp(M.H, M.HtypeBasis, [ny ny], 'H'); end
         catch E
             throw(E);
         end
         
         if ~isempty(M.StateName) && ~isequal(size(M.StateName),[nx 1])
             error(message('Control:ltiobject:ssProperties3','StateName'))
         end
         
         if ~isempty(M.StateUnit) && ~isequal(size(M.StateUnit),[nx 1])
            error(message('Control:ltiobject:ssProperties3','StateUnit'))
         end
         
         if ~isempty(M.X0) && ~isequal(getsize(M.X0), [nx 1])
             error(message('Control:ltiobject:ssProperties3','X0'))
         end
         
         if ~isempty(M.X0CovT.R) && ~isequal(size(M.X0CovT),[nx nx]) 
             LPVcore.error('LPVcore:general:InvalidProperty','X0CovT','idpack.FactoredCovariance object of size nx-by-nx','LPVcore.pmodel.lpvss')
         end
         
      end
      
      %--------------------------------------------------------------------
      function M = checkDataType(M)
         % Validates data type of A,B,C,D,K or A,B,C,D,G,H parameters.
         
         Aisemp = isempty(M.A) || isempty(M.A.Parameter(1).Value);
         Bisemp = isempty(M.B) || isempty(M.B.Parameter(1).Value);
         Cisemp = isempty(M.C) || isempty(M.C.Parameter(1).Value);
         Disemp = isempty(M.D) || isempty(M.D.Parameter(1).Value);
         Kisemp = isempty(M.K) || isempty(M.K.Parameter(1).Value);
         Gisemp = isempty(M.G) || isempty(M.G.Parameter(1).Value);
         Hisemp = isempty(M.H) || isempty(M.H.Parameter(1).Value);
         
         try
             if ~Aisemp; M.A = localCheckDataTypeProp(M.A, M.AtypeBasis, 'A'); end
             if ~Bisemp; M.B = localCheckDataTypeProp(M.B, M.BtypeBasis, 'B'); end
             if ~Cisemp; M.C = localCheckDataTypeProp(M.C, M.CtypeBasis, 'C'); end
             if ~Disemp; M.D = localCheckDataTypeProp(M.D, M.DtypeBasis, 'D'); end
             if ~Kisemp; M.K = localCheckDataTypeProp(M.K, M.KtypeBasis, 'K'); end
             if ~Gisemp; M.G = localCheckDataTypeProp(M.G, M.GtypeBasis, 'G'); end
             if ~Hisemp; M.H = localCheckDataTypeProp(M.H, M.HtypeBasis, 'H'); end
         catch E
             throw(E);
         end
         
         if ~iscellstr(M.StateName)
            LPVcore.error('LPVcore:general:InvalidProperty','StateName','cell string','LPVcore.pmodel.lpvss');
         end
         
         if ~iscellstr(M.StateUnit)
            LPVcore.error('LPVcore:general:InvalidProperty','StateUnit','cell string','LPVcore.pmodel.lpvss');
         end
         
         if ~isempty(M.X0); M.X0 = pmodel.checkParameter(M.X0,'X0'); end
         
         if ~isempty(M.X0CovT) && ~isa(M.X0CovT, 'idpack.FactoredCovariance')
             LPVcore.error('LPVcore:general:InvalidProperty','X0CovT','idpack.FactoredCovariance object','LPVcore.pmodel.lpvss')
         end
         
      end
      
      
      %--------------------------------------------------------------------
      function M = setABCDKGH(M, Prop, Val)
          
          if isa(Val,'pmatrix')
              % Pmatrix
              for i = 1:size(Val.Matrices,3)
                  M.(Prop).Parameter(i) = param.Continuous(Prop,Val.Matrices(:,:,i));
              end
              M.(Prop).Basis         = Val.BasisFunctions;
              M.SchedulingTimeMap    = Val.SchedulingTimeMap;
              M.([Prop,'typeBasis']) = getTypeBasis_(Val);
              
          elseif isempty(Val)
              % Empty parameter
              M.(Prop) = [];
              M.([Prop,'typeBasis']) = {};
          else
              % it is just a matrix
              M.(Prop).Parameter     = param.Continuous(Prop,Val);
              M.(Prop).Basis         = {};
              M.([Prop,'typeBasis']) = {0,[]};
          end
          
      end
      
      
      %--------------------------------------------------------------------
      function Val = typeNoiseModel(M)
          
          if isempty(M.G) && isempty(M.H)
              Val = 'innovation';
          else
              Val = 'general';
          end
          
      end
      
   end
   
end



%-------------------- LOCAL FUNCTIONS -------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function localCheckDataSizeProp(Val, typeBasis, sz, Prop)
    if numel(Val.Parameter) ~= numel(Val.Basis) || size(typeBasis,1) ~= numel(Val.Basis)
        LPVcore.error('LPVcore:general:InvalidInput',sprintf('The value of the "%s" property is inconsistent. Number of basis is different w.r.t. parameters.',Prop),'LPVcore.pmodel.lpvss');
    end

    for i = 1:numel(Val.Parameter)
        if ~isequal(getSize(Val.Parameter(i)),sz)
            LPVcore.error('LPVcore:general:InvalidProperty',Prop,sprintf('structure where %s.Parameter(%d) is of dimension %d-by-%d',Prop,i,sz(1),sz(2)),'LPVcore.pmodel.lpvss');
        end
    end
        
end



function Val = localCheckDataTypeProp(Val, typeBasis, Prop)

    if  ~isstruct(Val) || ~isequal({'Basis';'Parameter'},sort(fieldnames(Val)))
        LPVcore.error('LPVcore:general:InvalidProperty',Prop,'structure with fields Basis and Parameter','LPVcore.pmodel.lpvss');
    end

    if ~pmatrix.validateTypeBasis(typeBasis)
        LPVcore.error('LPVcore:general:InternalError',[mfilename,'.m']);
    end

    tmpType = [typeBasis{:,1}];

    basCon = tmpType == 0;
    basPol = tmpType == 1;
    basCus = tmpType == 9;

    if any( cellfun(@(x) ~isa(x, 'function_handle') , Val.Basis(logical(basPol+basCus)) )) || any( cellfun(@(x) ~isempty(x) , Val.Basis(basCon) ))
        LPVcore.error('LPVcore:general:InvalidProperty',sprinft('%s.Basis',Prop),'cell with function handles','LPVcore.pmodel.lpvss');
    end

    for i = 1:numel(Val.Parameter)
        Val.Parameter(i) = pmodel.checkParameter(Val.Parameter(i),Prop);
    end
end