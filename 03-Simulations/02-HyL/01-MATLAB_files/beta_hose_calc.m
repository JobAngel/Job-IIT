clc
close all
clear all

a_ccfoot = 0.0032;
a = a_ccfoot * 4.7572e-10;
b_cc = 0.1637;
b = b_cc * 3.2808e-6;
L = 0.5;
dia_in = 0.5;
dia =  dia_in*2.5e-2;
A = pi*dia^2/4; 
V0 = A*L;

zz = (V0 + b*L)/(a*L);

p = zeros(1,200);
V = zeros(1,200);
p(1) = 1e5;
V(1) = V0;
for m = 2:200
    p(m) = m*1e5;
    V(m) = V0 + a*p(m)*L + b*L;
    aux(m) = log(abs(V(m)/V(m-1)));
    beta(m) = (p(m)-p(m-1))/log(abs(V(m)/V(m-1)));
    beta2(m) = p(m) + zz;
end

plot(p,beta,p,beta2);