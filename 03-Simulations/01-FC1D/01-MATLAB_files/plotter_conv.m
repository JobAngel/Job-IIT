clear all
% close all
clc

test_name = 'd01985';
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

%%%
k = 0;

R_crit = 2;
L1 = 0.01;
L2 = 0.004;
L3 = 0.001;

[SSD1,f1] = SSDetector(R_crit,L1,L2,L3,pos,npt);
[SSD2,f2] = SSDetector(R_crit,L1,L2,L3,pB,npt);

for k = 2:npt
    if ((SSD1(k) + SSD2(k))>0)
    SSD(k) = 1;
    else SSD(k) = 0;
    end
end



ind = find(SSD);
tt = find(diff(ind)>2000);
tt2 = find(diff(tt)<500);
tt(tt2) = [];

if mod(length(tt),2) ~= 0
    tt(end) = [];
end

f_pos = zeros(size(pos));
f_pB = zeros(size(pB));

V = zeros(length(tt),1);
pBx = zeros(length(tt),1);

for k = 1:length(tt)
        area_x(:,k) = [time(ind(tt(k))) time(ind(tt(k)+1)) time(ind(tt(k)+1)) time(ind(tt(k)))];
        area_y(:,k) = [0 0 1 1];

        f_pos(ind(tt(k)):ind(tt(k)+1)) = (f1(ind(tt(k))) + f1(ind(tt(k)+1)))/2;
        f_pB(ind(tt(k)):ind(tt(k)+1)) = (f2(ind(tt(k))) + f2(ind(tt(k)+1)))/2;

        V(k) =  1e-3*((f1(ind(tt(k))) + f1(ind(tt(k)+1)))/2)*Ab;
        pBx(k) = (f2(ind(tt(k))) + f2(ind(tt(k)+1)))/2;
end

EV_m = zeros(length(tt)/2,1);
% EV_ft = zeros(length(tt)/2,1);
pBi_Pa = zeros(length(tt)/2,1);
% pBi_psi = zeros(length(tt)/2,1);

for k = 1:(length(tt)/2)
    dV_aux(k) = V(2*k) - V(2*k - 1);
    EV_m(k) = sum(dV_aux(1:k))/Lh;
%     EV_ft(k) = EV_m(k)*3.048e5;
    pBi_Pa(k) = mean(pBx((2*k - 1):(2*k)));
%     pBi_psi(k) = pBi_Pa(k)*14.5/1e5;
end
% Inclusion of origin point
% EV_m = [0; EV_m];
% EV_ft = [0; EV_ft];
% pBi_Pa = [0; pBi_Pa];
% pBi_psi = [0; pBi_psi];

b1 = polyfit(pBi_Pa,EV_m,1);
% b2 = polyfit(pBi_psi,EV_ft,1);
EV_calc = polyval(b1,pBi_Pa);
% EV_calc2 = polyval(b2,pBi_psi);

count = 1;
for p=0:10e5:2e8
    beta(count) = p + ((Ah + b1(2))/b1(1)); 
    count = count+1;
end


% PLOTTING
%%%%%%%%%%

ax1 = subplot(2,2,1);
plot(time,pos,time,f1,time,f_pos);
axx1 = gca;
patch(area_x,axx1.YLim(2)*area_y,'k','EdgeColor','none')
alpha(0.1);

for k=1:length(tt)
    cent(k) = time(ind(tt(k)));
    text(cent(k),10,num2str(k));
end
% legend('pos')

ax2 = subplot(2,2,3);
plot(time,pA,time,pB,time,f_pB);
axx2 = gca;
patch(area_x,axx2.YLim(2)*area_y,'k','EdgeColor','none')
alpha(0.1);

for k=1:length(tt)
    cent(k) = time(ind(tt(k)));
    text(cent(k),1e7,num2str(k));
end

ax3 = subplot(2,2,2);
scatter(pBi_Pa,EV_m);hold on
plot(pBi_Pa,EV_calc);
text(pBi_Pa(5,1),EV_calc(3,1),['y [m3/m] = ',num2str(b1(1)),' * p [Pa] + ',num2str(b1(2))]);

ax4 = subplot(2,2,4);
p = 0:10e5:2e8;
plot(p, beta)

linkaxes([ax1, ax2],'x')
