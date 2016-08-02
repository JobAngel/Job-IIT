clear all

test_name = 'd01984';
filename = test_name;
[D,vars,freq] = clmcplot_convert(test_name);

cut_time = 350; % period seconds
npt = freq * cut_time + 1; % number of points
Dc = D(1:npt,:);


% Hydraulic system data
% ---------------------
d_p = 16e-3;          
% m - piston diameter
d_r = 10e-3;                % m - rod diameter
L = 80e-3;                  % m - stroke
Aa = pi*d_p^2/4;            % m2 - Area of side A
Ab = (d_p^2 - d_r^2)*pi/4;  % m2 - Area of side B
r = Ab/Aa;                  % Ratio
d_h = 5e-3;                 % m - Hose diameter
Ah = pi*d_h^2/4;            % m2 - Hose section
Lh = 560e-3;                % m2 - Hose length

% Data vectors
% ------------
time = Dc(:, 1);
pos = Dc(:, 2);
vel = Dc(:, 3);
pA = 1e5*Dc(:, 4);
pB = 1e5*Dc(:, 5);
pS = 1e5*Dc(:, 6);
pT = 1e5*Dc(:, 7);
force = Dc(:, 8);
valve = Dc(:, 9);

% Vectors initialization
% ---------------------
ff_pos(1) = 0;
ff_pos2(1) = 0;

ff_pA(1) = 0;
ff_pB(1) = 0;
ff_pS(1) = 0;
ff_pT(1) = 0;
ff_pL(1) = 0;
ff_vel(1) = 0;
af_pos(1) = 0; % for average filter

V(1) = Lh*Ah + Ab*L;
dV(1) = 0;

%%%
k = 0;

R_crit = 2;

L1 = 0.01;
L2 = 0.004;
L3 = 0.002;

fx(1) = 0;
fy(1) = 0;
sigma1(1) = 0;
sigma2(1) = 0;
sigma11(1) = 0;
sigma22(1) = 0;
S1(1) = 0;
S2(1) = 0;
S11(1) = 0;
S22(1) = 0;
R(1) = 1;
R2(1) = 1;
SState(1) = 0;
SState2(1) = 0;
SSD = 0;

x = pos;
y = pA;

for k = 2:npt

    fx(k) =  L1*x(k) + (1 - L1)*fx(k-1);
    sigma1(k) = L2*(x(k) - fx(k-1))^2 + (1 - L2)*sigma1(k-1);
    sigma2(k) = L3*(x(k) - x(k-1))^2 + (1 - L3)*sigma2(k-1);
    S1(k) = sigma1(k)*(2 - L1)/2; 
    S2(k) = sigma2(k)/2;
    R(k) = S1(k)/S2(k);
    if (R(k) > R_crit)
        SState(k) = 1;
    else SState(k) = 0;
    end
    
    fy(k) =  L1*y(k) + (1 - L1)*fy(k-1);
    sigma11(k) = L2*(y(k) - fy(k-1))^2 + (1 - L2)*sigma11(k-1);
    sigma22(k) = L3*(y(k) - y(k-1))^2 + (1 - L3)*sigma22(k-1);
    S11(k) = sigma11(k)*(2 - L1)/2; 
    S22(k) = sigma22(k)/2;
    R2(k) = S11(k)/S22(k);
    if (R2(k) > R_crit)
        SState2(k) = 1;
    else SState2(k) = 0;
    end
   
    if ((SState(k) + SState2(k))>0)
    SSD(k) = 1;
    else SSD(k) = 0;
    end
end

ind = find(~SSD);
dind = diff(ind); tt = find(dind>1);

count = 0;
for k=1:(length(tt)-1)
    area_x(:,k) = [time(ind(tt(k)+1)) time(ind(tt(k+1))) time(ind(tt(k+1))) time(ind(tt(k)+1))];
    area_y(:,k) = [0 0 1 1];
%     c = [0 .25 .25];
end
   
ax1 = subplot(2,1,1);
plot(time,pos,time,fx,'LineWidth',2);
axx1 = gca;
patch(area_x,axx1.YLim(2)*area_y,'k','EdgeColor','none')
alpha(0.1);
legend('pos')

ax2 = subplot(2,1,2);
plot(time,pA,time,pB,time,pS, time,pT);
axx2 = gca;
patch(area_x,axx2.YLim(2)*area_y,'k','EdgeColor','none')
alpha(0.1);
legend('pA','pB','pS','pT')

linkaxes([ax1, ax2],'x')
