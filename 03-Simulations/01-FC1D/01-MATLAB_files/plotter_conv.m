clear all

test_name = 'd01984';
filename = test_name;
[D,vars,freq] = clmcplot_convert(test_name);

cut_time = 350; % period seconds
npt = freq * cut_time + 1; % number of points
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
time = Dc(1:npt, 1);        
pos = 1e-3*Dc(1:npt, 2);    
vel = 1e-3*Dc(1:npt, 3);    
pA = 1e5*Dc(1:npt, 4);      
pB = 1e5*Dc(1:npt, 5);
pS = 1e5*Dc(1:npt, 6);
pT = 1e5*Dc(1:npt, 7);
force = Dc(1:npt, 8);
valve = Dc(1:npt, 9);

% Filter
% ------
alpha = 0.995; % first order filter coefficient

% Vectors initialization
% ---------------------
ff_pos(1) = 0;
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

for k = 2:npt
    ff_pos(k) = (1-alpha)*pos(k) + alpha*ff_pos(k-1);       % first order filtered position
    af_pos(k) = af_pos(k-1) + (1/k)*(pos(k) - af_pos(k-1)); % average filtered position
    ff_vel(k) = (1-alpha)*vel(k) + alpha*ff_vel(k-1);       % first order filtered velocity

    V(k) = V(k-1) - Ab*(L-ff_pos(k));
%     beta(k) = (pB(k) - pB(k-1))/log((V(k)/V(k-1)));
%     beta(k) = (1-alpha2)*beta(k) + alpha*beta(k-1);
%     
%     dV(k) = V(k) - V(k-1);
%     Vo(k) = Vo(k-1) + dV(k);
%     VodV(k) = Vo(k)/dV(k);
%     dpB(k) = pB(k)-pB(k-1);
%     
%     beta(k) = VodV(k)/dpB(k);
    
    ff_pA(k) = (1-alpha)*pA(k) + alpha*ff_pA(k-1);
    ff_pB(k) = (1-alpha)*pB(k) + alpha*ff_pB(k-1);
    ff_pS(k) = (1-alpha)*pS(k) + alpha*ff_pS(k-1);
    ff_pT(k) = (1-alpha)*pT(k) + alpha*ff_pT(k-1);
    pL(k) = Aa*(pA(k) - r*pB(k));
    ff_pL(k) = (1-alpha)*pL(k) + alpha*ff_pL(k-1);
end

fit_pos = fit(time,ff_pos','poly3');
coeff = coeffvalues(fit_pos);

ax1 = subplot(3,1,1);
plot(time,ff_pos);
legend('filtered pos')

ax2 = subplot(3,1,2);
plot(time,ff_pA,time,ff_pB,time,(r*ones(npt,1)),time,ff_pL);
legend('filtered pA','filtered pB','pL','filtered pL')

ax3 = subplot(3,1,3);
plot(time,V);
legend('V')

linkaxes([ax1, ax2, ax3],'x')

%%
% ax2 = subplot(5,1,2);
% plot(time,beta);
% legend('beta')

% ax3 = subplot(5,1,3);
% plot(time,pL,'r',time,zeros(npt,1),'b');
% legend('pL','zero')
ax3 = subplot(5,1,3);
plot(time_1,Vo,time_1,V);
legend('Vo','V')

% 
ax4 = subplot(5,1,4);
plot(time_1,dV);
legend('dV')

% ax5 = subplot(5,1,5);
% plot(time,valve);
% legend('valve')
ax5 = subplot(5,1,5);
plot(time_1,dpB);
legend('dp')

% plot(time,pS,'r',time,pT,'b');
% legend('pS','pT')

linkaxes([ax1, ax2, ax3, ax4, ax5],'x')
% plot(time,pos,'r',time,f_pos,'b',time,af_pos,'g');

