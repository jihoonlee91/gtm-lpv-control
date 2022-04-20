clear
%% plant
spec_data % load specification
aero_data % load aerodynamic data

V_grid = 20;
h_grid = 300;
eta1_grid = 0:0.25:1;
eta2_grid = 0:0.25:1;

V = pgrid('V',V_grid,[-5 5]);
eta1 = pgrid('eta1',eta1_grid,[-0.2 0.2]);
eta2 = pgrid('eta2',eta2_grid,[-0.2 0.2]);

wa = 2*pi*13;
za = 0.6;

figure('position',[0 0 490 500])
tic
for ii = 1:numel(V_grid)
    for jj = 1:numel(h_grid)
        for nn = 1:numel(eta1_grid)
            for mm = 1:numel(eta2_grid)
                [Gss4,alp_trim(nn,mm),delt_trim(nn,mm),dele_trim(nn,mm)] = lti_model(V_grid(ii),h_grid(jj),eta1_grid(nn),eta2_grid(mm));
                Zalpha = Gss4.A(2,2)*V_grid(ii);
                Zdelta = Gss4.B(2,2)*V_grid(ii);
                Malpha = Gss4.A(3,2);
                Mdelta = Gss4.B(3,2);
                A = [Zalpha/V_grid(ii),1;
                    Malpha,0];
                B = [Zdelta/V_grid(ii);
                    Mdelta];
                C = [1,0;
                    0,1;
                    Zalpha,0];
                D = [0;
                    0;
                    Zdelta];
                Gss(:,:,nn,mm) = ss(A,B,C,D); % short-period mode
                pzplot(Gss(:,:,nn,mm),'k')
                hold on
            end
        end
    end
end
toc

grid on
legend
ylim([0 9])
xlim([-8 0])

for nn = 1:numel(eta1_grid)
    for mm = 1:numel(eta2_grid)
        EEr(nn,mm) = [1,0]*real(eig(Gss(:,:,nn,mm)));
        EEi(nn,mm) = [1,0]*imag(eig(Gss(:,:,nn,mm)));
    end
end

for nn = 1:numel(eta1_grid)
    plot(EEr(:,nn),EEi(:,nn),'k:')
end

for mm = 1:numel(eta2_grid)
    plot(EEr(mm,:),EEi(mm,:),'k:')
end


% h = findobj(gca, 'type', 'line');
% set(h, 'markersize', 9)
% text(real(squeeze(pole(Gss))) - 0.1, imag(squeeze(pole(Gss))) + 0.1, 'Pole')
% annotation('textarrow',[-2.31,-2.99],[6.11,3.92],'String',' sweeping back ')
% annotation('textarrow',x,y,'String',' y = x ')