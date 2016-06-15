function output = referenceGenerator(input)

time = input(1);
q = [input(2) input(3)]';
dq = [input(4) input(5)]';

%Constant position
des_q = 3.14/180 * [(45+5*sin(4*pi*time)) (-90-10*sin(4*pi*time))]';
des_dq = 3.14/180 * [(4*pi*5*cos(4*pi*time)) (-4*pi*10*cos(4*pi*time))]';

des_Xf(1) = 0.0;
des_Xf(2) = -0.5 + 0*0.2*cos(0.25*pi*time);

%des_q = hylInvKinematics(des_Xf);
%des_dq = [0.0 0.0;]';

output = [des_q; des_dq];

end