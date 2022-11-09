rng(3);

p = preal('p', 'dt');

N = 3;
np = 1;

Delta = 0.1 * p;
G = drss(20, 2, 2);
Dzw = -0.1;
G.D = [Dzw, 0; 0, 0];
sys = lpvlfr(Delta, G);

pVec = linspace(-1, 1, N);
p = cell(1, N);
for i=1:N
    p{i} = pVec(i);
end

G = extractLocal(sys, p);

[pNext, nugapEst, results] = bayesops(G, p);

%% Plot results
N = 20;

P = linspace(-1, 1, N);
Q = linspace(-1, 1, N);
F = NaN(N, N);
F_hat = NaN(N, N);
for i=1:N
    for j=1:N
        [~, F(i, j)] = gapmetric(extractLocal(sys, P(i)), ...
            extractLocal(sys, Q(j)));
        F_hat(i, j) = nugapEst(P(i), Q(j));
    end
end
subplot(1, 2, 1);
surf(P, Q, F);
xlabel('p');
ylabel('q');
zlabel('f(p, q)');

subplot(1, 2, 2);
surf(P, Q, F_hat);
xlabel('p');
ylabel('q');
zlabel('f(p, q)');
