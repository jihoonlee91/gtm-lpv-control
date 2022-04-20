function [Gss,alp_trim,delt_trim,dele_trim] = lti_model(V,h,eta1,eta2)
eta = [eta1;eta2];
[~,u_trim,alp_trim,~] = trim_calc(zeros(1,3),V,h,eta);
delt_trim = u_trim(1);
dele_trim = u_trim(3);

wind_trim = [V;alp_trim;0];
angle_trim = [0;alp_trim;0];
omega_trim = zeros(3,1);
pos = [0;0;-h];
[A_,B_,~,~] = lin_mod([wind_trim;omega_trim;angle_trim;pos],[delt_trim;dele_trim;0;0],eta);
% V alpha q theta

% V alpha q gamma
% A_(4,:) = A_(4,:)-A_(2,:); B_(4,:) = B_(4,:)-B_(2,:);

% V alpha gamma q
% A = A_([1 2 4 3],[1 2 4 3]);
% B = B_([1 2 4 3],:);

% C = [1 0 0 0; 0 0 1 0];
% D = zeros(2);

% A_(2,:) = A_(4,:)-A_(2,:); B_(2,:) = B_(4,:)-B_(2,:);
Gss = ss(A_,B_,eye(4),zeros(4,2));
end