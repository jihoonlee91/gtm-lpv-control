spec_data % load specification
aero_data % load aerodynamic data

tas=10:2:50;

% Allocate variables to plot

figure('position',[100 50 1200 650])

for kk = 1:2 % span
for jj = 1:2 % sweep

    alp_trim=zeros(length(tas),1);
    dele_trim=zeros(length(tas),1);
    delt_trim=zeros(length(tas),1);

    for ii = 1:length(tas) % true airspeed
    
    h0 = 300; % initial altitude [m]
    eta0 = [kk-1;jj-1]; % morphing parameter [span; sweep]

    alp_guess = 0; % guess for trimmed AoA [rad]
    delt_guess = 0; % guess for trimmed throttle
    dele_guess = 0; % guess for trimmed elevator
    z_guess = [alp_guess;delt_guess;dele_guess]; % guess for trimmed state and control

    [x_trim,u_trim,alp_trimm,Err] = trim_calc(z_guess,tas(ii),h0,eta0); % trim calculation
    delt_trim(ii) = u_trim(1)*100;
    dele_trim(ii) = u_trim(3)*180/pi;
    alp_trim(ii) = alp_trimm*180/pi;

    if Err > 1e-3 % skip
        delt_trim(ii) = NaN;
        dele_trim(ii) = NaN;
        alp_trim(ii) = NaN;
    end
    end

    % make tas trim point plot
    subplot(2,2,2*(kk-1)+jj),
    [ax,h1,h2]=plotyy(tas,[dele_trim,alp_trim],tas,delt_trim); grid on
    set(ax,{'YColor'},{'blue';'red'});
    set([h1;h2],{'Color'},{'blue';'blue';'red'});
    set([h1;h2],{'Marker'},{'o';'s';'>'})
    set([h1;h2],{'MarkerFaceColor'},{'blue';'none';'red'});
    % title('Trimmed level flight by tas');
    set([h1;h2],{'Marker'},{'o';'s';'>'});
    ylabel(ax(1),[sprintf('%c', char(9679)),' \delta_e (deg), ',sprintf('%c', char(9633)),' \alpha (deg)']);
    ylabel(ax(2),'Throttle (%)');
    xlabel('tas (m/s)')

    ylim(ax(1),[-10 10])
    ylim(ax(2),[0 100])
    xlim(ax(1),[10 50])
    xlim(ax(2),[10 50])

    yticks(ax(1),[-10 -5 0 5 10])
    yticks(ax(2),[0 25 50 75 100])
    yticklabels(ax(1),{'-10','-5','0','5','10'})
    yticklabels(ax(2),{'0','25','50','75','100'})

    xticks(ax(1),[10 15 20 25 30 35 40 45 50])
    xticklabels(ax(1),{'10','15','20','25','30','35','40','45','50'})
    xticks(ax(2),[10 15 20 25 30 35 40 45 50])
    xticklabels(ax(2),{'10','15','20','25','30','35','40','45','50'})

    ax = gca;
    outerpos = ax.OuterPosition;
    ti = ax.TightInset; 
    left = outerpos(1) + ti(1);
    bottom = outerpos(2) + ti(2);
    ax_width = outerpos(3) - ti(1) - ti(3);
    ax_height = outerpos(4) - ti(2) - ti(4);
    ax.Position = [left bottom ax_width ax_height];
    
    title(['\eta = [',num2str(eta0(1)),' ',num2str(eta0(2)),']^T'])

end
end

% set(gcf,'units','normalized')
% set(gcf,'papersize',[50 40])
% print([pwd,'\result\trim'],'-dpdf')