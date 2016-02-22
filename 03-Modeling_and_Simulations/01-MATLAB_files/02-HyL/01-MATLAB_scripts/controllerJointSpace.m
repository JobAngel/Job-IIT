function output = controllerJointSpace(input)

%Proportial and Derivative gains (gain matrices)
global Kp Kd;

%Desired and actual joint position vectors
des_q = [input(1) input(2)]';
q = [input(5) input(6)]';

%Desired and actual joint position vectors
des_dq = [input(3) input(4)]';
dq = [input(7) input(8)]';

%Control action
output = Kp * (des_q - q) + Kd * (des_dq -dq);

end