clc
close all
clear all

p0 = 1e5;
n = 1.4;
B = 1.5e9;
a = 1;
b = 1;

for R = 0.01*[0,0.5,2,10]
   for p = 1e5:1e5:120e5

% COMPRESSION ONLY MODELS       
       beta(a,b) = B*((p/p0)^n + R) / ((p/p0)^n + (R*B/(n*p))); % Watton model
%        beta(a,b) = B / (1 + R*(B/(n*p) - 1)); % Merrit model
%        beta(a,b) = (R*(p0/p)^(1/n) + 1 - R)^2 / (R*(p0/p)^(1/n)/(n*p) + (1-R)/B); % Nykanen model
%        beta(a,b) = (R*(p0/p)^(1/n) + 1 - R) / (R*(p0/p)^(1/n)/(n*p) + (1-R)/B); % Cho model
     
       b = b+1;
   end
   a = a+1;
   b = 1;
end
R = [0,0.5,2,10];
ps = 1e5:1e5:120e5;
for a=1:length(R)
    figure (1)
    plot(ps,beta(a,:)); hold on
end
