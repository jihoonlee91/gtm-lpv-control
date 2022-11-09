function sys = lft_(sys1, sys2, nu, ny)
%LFT_ Performs LFT interconnection
%   Interconnection as shown in diagram below:
%
%             +-----------------------------+
%             |                             |
%             | +---------+                 |
%        +------+Delta1(p)+<----------------------+
%        |    | +---------+                 |     |
%        |    |                             |     |
%        |    |                 +---------+ |     |
%     +-------------------------+Delta2(p)+<----------+
%     |  |    |                 +---------+ |     |   |
%     |  |    |                             |     |   |
%     |  |    +-----------------------------+     |   |
%     |  |                                        |   |
%     |  |                                        |   |
%     |  |                                        |   |
%     |  |    +-----------------------------+     |   |
%     |  |    |                             |     |   |
%     |  |    |       +------------+        |     |   |
%     |  +----------->+            +--------------+   |
%     |       |       |            |        |         |
% +------------------>+     G1     +----------------------->
%     |       |       |            |        |         |
%     |   From G2 +-->+            +--> To G2         |
%     |       |       +------------+        |         |
%     |       |                             |         |
%     |       |       +------------+        |         |
%     +-------------->+            +------------------+
%             |       |            |        |
%         From G1 +-->+     G2     +--> To G1
%             |       |            |        |
% +------------------>+            +----------------------->
%             |       +------------+        |
%             |                             |
%             +-----------------------------+
%
%

delta = blkdiag(sys1.Delta, sys2.Delta);
blksys = append(sys1.G, sys2.G);
nw1 = sys1.Nw;
nw2 = sys2.Nw;
nz1 = sys1.Nz;
nz2 = sys2.Nz;
ny1 = size(sys1.G, 1);
nu1 = size(sys1.G, 2);
ny2 = size(sys2.G, 1);
nu2 = size(sys2.G, 2);

% Connect last ny outputs of G1 to nw2+1:nw2+ny inputs of G2.
connections = [ nu1+((nw2+1):(nw2+ny))', ((ny1-ny+1):ny1)' ];
% Connect nz2+1:nz2+nu outputs of G2 to last nu inputs of G1.
connections = [connections; ...
    ((nu1-nu+1):nu1)', ny1+((nz2+1):(nz2+nu))'];

g = connect(blksys, connections, ...
    [1:nu1-nu, nu1+1:nu1+nw2, nu1+nw2+ny+1:nu1+nu2], ...
    [1:ny1-ny, ny1+1:ny1+nz2, ny1+nz2+nu+1:ny1+ny2]);

nf1 = nu1 - nw1 - nu;
ng1 = ny1 - nz1 - ny;

% Switch inputs for B-matrix
B1 = g.B(:, [1:nw1, (nw1+nf1+1):(nw1+nf1+nw2)]);
B2 = g.B(:, [(nw1+1):(nw1+nf1), (nw1+nf1+nw2+1):end]);

% Switch outputs for C-matrix
C1 = g.C([1:nz1, (nz1+ng1+1):(nz1+ng1+nz2)], :);
C2 = g.C([(nz1+1):(nz1+ng1), (nz1+ng1+nz2+1):end], :);

% Switch inputs and outputs for D-matrix
D11 = g.D([1:nz1, (nz1+ng1+1):(nz1+ng1+nz2)],[1:nw1, (nw1+nf1+1):(nw1+nf1+nw2)]);
D12 = g.D([1:nz1, (nz1+ng1+1):(nz1+ng1+nz2)],[(nw1+1):(nw1+nf1), (nw1+nf1+nw2+1):end]);
D21 = g.D([(nz1+1):(nz1+ng1), (nz1+ng1+nz2+1):end],[1:nw1, (nw1+nf1+1):(nw1+nf1+nw2)]);
D22 = g.D([(nz1+1):(nz1+ng1), (nz1+ng1+nz2+1):end],[(nw1+1):(nw1+nf1), (nw1+nf1+nw2+1):end]);

g.B = [B1, B2];
g.C = [C1; C2];
g.D = [D11,D12;D21,D22];

sys = sys1;  % TODO: always sys1?
sys.Delta_ = delta;
sys.G_ = g;
sys.SchedulingTimeMap_ = delta.timemap;
sys.InputName = [sys1.InputName(1:nf1); ...
    sys2.InputName(ny+1:end)];
sys.InputUnit = [sys1.InputUnit(1:nf1); ...
    sys2.InputUnit(ny+1:end)];
sys.InputDelay = [sys1.InputDelay(1:nf1); ...
    sys2.InputDelay(ny+1:end)];
sys.OutputName = [sys1.OutputName(1:ng1); ...
    sys2.OutputName(nu+1:end)];
sys.OutputUnit = [sys1.OutputUnit(1:ng1); ...
    sys2.OutputUnit(nu+1:end)];
sys.StateName = [sys1.StateName; sys2.StateName];
sys.StateUnit = [sys1.StateUnit; sys2.StateUnit];

end

