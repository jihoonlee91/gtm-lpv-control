close all
clear
load('matlab.mat')

%% bodeplot
fig2 = figure('position',[-958 -158+474 713 374]);
% bodemag(sys(1,1),{0.1,1000},'b')
% hold on
% bodemag(CL(1,1),{0.1,1000},'r--')
% legend('Open-loop','Closed-loop','location','northeast')
% grid minor
% hold off
% drawnow

[~,~,wout] = bode(Gss(:,:,1,1),{0.1,1000});
for nn = 1:numel(eta1_grid)
    for mm = 1:numel(eta2_grid)
        [magsys,~,~] = bode(Gss(:,:,nn,mm),wout);
        magOL(:,(nn-1)*3+mm) = reshape(magsys(1,1,:),[numel(wout),1]);
        [magSYS,~,~] = bode(lpvsubs(CL,{'eta1','eta2','eta1Dot','eta2Dot'},[eta1_grid(nn);eta2_grid(mm);0;0]),wout);
        magCL(:,(nn-1)*3+mm) = reshape(magSYS(1,1,:),[numel(wout),1]);
    end
end
h1 = semilogx(wout,mag2db(magOL),'b');
hold on
h2 = semilogx(wout,mag2db(magCL),'--r');
% lpvinterp(CL,'eta1',eta1_grid(nn),'eta2',eta2_grid(mm))

% title('Bode Diagram (from \alpha_c to \alpha)')
xlabel('Frequency, rad/s')
ylabel('Magnitude, dB')
legend([h1(1),h2(1)],'Open-Loop','Closed-Loop')
% axis([1e-1 1e2 -30 50])
grid on

%% step response
fig2 = figure('position',[-958 -158 913 474]);
opt = stepDataOptions('StepAmplitude',pi/180);

for nn = 1:numel(eta1_grid)
    for mm = 1:numel(eta2_grid)
        GCL(:,:,nn,mm) = lpvsubs(CL,{'eta1','eta2','eta1Dot','eta2Dot'},[eta1_grid(nn);eta2_grid(mm);0;0]);
        [y,t] = step(GCL(1,1,nn,mm),opt,0:0.01:2.5);
        Y(:,(nn-1)*3+mm) = y*180/pi;
    end
end

plot(t,Y(:,1),'b','linewidth',2), hold on
plot(t,Y(:,4),'b--','linewidth',2)
plot(t,Y(:,7),'b:','linewidth',2)
plot(t,Y(:,2),'r','linewidth',2)
plot(t,Y(:,5),'r--','linewidth',2)
plot(t,Y(:,8),'r:','linewidth',2)
plot(t,Y(:,3),'k','linewidth',2)
plot(t,Y(:,6),'k--','linewidth',2)
plot(t,Y(:,9),'k:','linewidth',2)
grid on
xlabel('Time, s')
ylabel('Angle of Attack, deg')
legend('location','southeast',...
    '\eta=(0.0,0.0)',...
    '\eta=(0.5,0.0)',...
    '\eta=(1.0,0.0)',...
    '\eta=(0.0,0.5)',...
    '\eta=(0.5,0.5)',...
    '\eta=(1.0,0.5)',...
    '\eta=(0.0,1.0)',...
    '\eta=(0.5,1.0)',...
    '\eta=(1.0,1.0)')