function out = SSDetector(R_crit,L1,L2,L3,x,npt)

% Parameters initialization 
dim = size(x);
fx = zeros(dim);
sigma1 = zeros(dim);
sigma2 = zeros(dim);
S1 = zeros(dim);
S2 = zeros(dim);
R = zeros(dim);
SState = zeros(dim);

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
end

out = SState;