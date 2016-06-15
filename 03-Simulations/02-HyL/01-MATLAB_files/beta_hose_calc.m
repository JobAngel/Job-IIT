clc
close all
clear all

a_ccfoot =  0.0032;
a = a_ccfoot * 4.7572e-10;
b_cc = 0.1637;
b = b_cc * 3.2808e-6;
L = 0.5;
dia_in = 0.5; % diameter of the hose
dia =  dia_in*2.5e-2;
A = pi*dia^2/4; 
V0 = A*L; % Initial volume of the hose

P0 = 100e5; % precharge pressure of the accumulator;
V0_acc = 0.04e-3;
n = 1.4;

zz = (V0 + b*L)/(a*L);

E = 2.069e8;
vu = 0.3;
Do = 6e-3;
t = 1e-3;
do = 4e-3;
Bc1 = ((2/E)*((Do^2 + do^2)/(Do^2 - do^2) + vu))^-1;
Bc2 = ((2/E)*((Do/(2*t))*(1 + t/(Do - t)) - (1 - vu)))^-1;

mf = 5.6;
beta0 = 1.47e9;

p = zeros(1,200);
V = zeros(1,200);
V_acc = zeros(1,200);

p(1) = 1e5;
V(1) = V0;

for m = 2:200
    p(m) = m*1e5;
    V(m) = V0 + a*p(m)*L + b*L;
    V_acc(m) = V0_acc*(1 - (P0/p(m))^(1/n));
    
    % beta fluid
    betaf1(m) = beta0 + mf*p(m);
    betaf2(m) = beta0 + (1+(mf - 1)*p(m)/(beta0))*(1 + mf*p(m)/beta0);
    
    % beta hose
    beta(m) = (p(m)-p(m-1))/log(abs(V(m)/V(m-1)));
    beta2(m) = p(m) + zz;
    
    % beta acc
    beta_acc(m) = (p(m)-p(m-1))/log(abs(V_acc(m)/V_acc(m-1)));
    beta_acc2(m) = n*p(m)*((p(m)/P0)^(1/n) - 1);
    if p<=P0
        beta_acc(m) = 0;
    else beta_acc(m) = beta_acc(m);
    end
end

plot(p,beta,p,beta2)%,p,beta_acc,p,beta_acc2,p,betaf2);