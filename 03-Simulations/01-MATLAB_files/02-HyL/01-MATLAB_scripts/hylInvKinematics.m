function output = hylInvKinematics(des_Xf)

l0 = 0.09; %Base to HFE distance
l1 = 0.35; %HFE to KFE distance
l2 = 0.33; %KFE to foot distance
l3 = 0.02; %Foot radius (not used)

hfe2foot = sqrt( des_Xf(1)^2 + (des_Xf(2) - -l0)^2);

des_q(2) = - 3.1415 + acos((l1^2 + l2^2 - hfe2foot^2) /(2 * l1 * l2));

des_q(1) = -asin(des_Xf(1)/hfe2foot) +...
            acos((l1^2 + hfe2foot^2 - l2^2) / (2 * l1 * hfe2foot));

output = [des_q(1) des_q(2)]';

end