clear all
close all
clc

test_name = 'd01984';
filename = test_name;
[D,vars,freq] = clmcplot_convert(test_name);

cut_time = 350;             % period seconds
npt = freq * cut_time + 1;  % number of points
Dc = D(1:npt,:);


% Hydraulic system data
% ---------------------
d_p = 16e-3;                % m - piston diameter
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

V(1) = Lh*Ah + Ab*L;
dV(1) = 0;

%%%
k = 0;

R_crit = 2;
L1 = 0.01;
L2 = 0.004;
L3 = 0.002;

SSD1 = SSDetector(R_crit,L1,L2,L3,pos,npt);
SSD2 = SSDetector(R_crit,L1,L2,L3,pA,npt);

for k = 2:npt
    if ((SSD1(k) + SSD2(k))>0)
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
end
   
ax1 = subplot(2,1,1);
plot(time,pos);
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
