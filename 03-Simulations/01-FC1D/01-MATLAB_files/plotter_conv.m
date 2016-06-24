clear all

test_name = 'd01984';
filename = test_name;
[D,vars,freq] = clmcplot_convert(test_name);

cut_time = 300; % period seconds
npt = freq * cut_time + 1; % number of points
Dc = D(1:npt,:);
alpha = 0.995;

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

k = 0;

vartheta_crit = 2;
lambda1 = 0.2;
lambda2 = 0.05;
lambda3 = 0.09;

fx(1) = 0;
sigma1(1) = 0;
sigma2(1) = 0;
S1(1) = 0;
S2(1) = 0;
vartheta_stat(1) = 1;
SState(1) = 0;

dx = diff(pos)*freq; dx(end+1) = dx(end);
x = pA*1e-6;

for k = 2:npt
    fx(k) =  lambda1*x(k) + (1 - lambda1)*fx(k-1);
    sigma1(k) = lambda2*(x(k) - fx(k-1))^2 + (1 - lambda2)*sigma1(k-1);
    sigma2(k) = lambda3*(x(k) - x(k-1))^2 + (1 - lambda3)*sigma2(k-1);
    S1(k) = sigma1(k)*(2 - lambda1)/2; 
    S2(k) = sigma2(k)/2;
    vartheta_stat(k) = S1(k)/S2(k);
    if (vartheta_stat(k) > vartheta_crit)
        SState(k) = 1;
    else SState(k) = 0;
    end
end

plot(time,fx,time, SState);

%%

fA(1) = 0;
fB(1) = 0;

for k = 2:npt
    ff_pos(k) = (1-alpha)*pos(k) + alpha*ff_pos(k-1);       % first order filtered position
   
    af_pos(k) = af_pos(k-1) + (1/k)*(pos(k) - af_pos(k-1)); % average filtered position
    ff_vel(k) = (1-alpha)*vel(k) + alpha*ff_vel(k-1);       % first order filtered velocity

    V(k) = Ab*ff_pos(k);

    ff_pA(k) = (1-alpha)*pA(k) + alpha*ff_pA(k-1);
    ff_pB(k) = (1-alpha)*pB(k) + alpha*ff_pB(k-1);
    ff_pS(k) = (1-alpha)*pS(k) + alpha*ff_pS(k-1);
    ff_pT(k) = (1-alpha)*pT(k) + alpha*ff_pT(k-1);
    pL(k) = Aa*(pA(k) - r*pB(k));
    ff_pL(k) = (1-alpha)*pL(k) + alpha*ff_pL(k-1);
    fA(k) = ff_pA(k) * Aa;
    fB(k) = ff_pB(k) * Ab;
end

f_dx(1) = 0;
SSD(1) = 0;
vcrit = 0.025;

dx = diff(ff_pos)*freq; dx(end+1) = dx(end);
for k = 2:npt
    f_dx(k) = (1-alpha)*dx(k) + alpha*f_dx(k-1);       % first order filtered position
    if (f_dx(k)>vcrit) || (f_dx(k)< -vcrit)
        SSD(k) = 0;
    else SSD(k) = 1;
    end
end

ind = find(SSD);

dpA = diff(ff_pA); dpA(end+1) = dpA(end);
dpB = diff(ff_pB); dpB(end+1) = dpB(end);
dpS = diff(ff_pS); dpS(end+1) = dpS(end);
dpT = diff(ff_pT); dpT(end+1) = dpT(end);

ax1 = subplot(3,1,1);
plot(time,pos,time, ff_pos);
legend('pos','fpos');

ax2 = subplot(3,1,2);
plot(time,f_dx);
legend('dx')

ax3 = subplot(3,1,3);
plot(time,SSD);
legend('SSD')
linkaxes([ax1, ax2, ax3],'x')

%%

% m=0;
% for m = 2:npt2
%     ff_pos2(m) = (1-alpha2)*pos2(m) + alpha2*ff_pos2(m-1);       % first order filtered position
% end

fit_pos = fit(time,ff_pos','poly3');
coeff = coeffvalues(fit_pos);

ax1 = subplot(3,1,1);
% plot(fit_pos,time,ff_pos);
plot(time,ff_pos,time,af_pos);
legend('1st order filt','@filt')

ax2 = subplot(3,1,2);
% plot(time,ff_pA,time,ff_pB,time,(r*ones(npt,1)),time,ff_pL);
% legend('filtered pA','filtered pB','pL','filtered pL')
plot(time,pB);
legend('dpB')

ax3 = subplot(3,1,3);
plot(time,V);
legend('V')

linkaxes([ax1, ax2, ax3],'x')

