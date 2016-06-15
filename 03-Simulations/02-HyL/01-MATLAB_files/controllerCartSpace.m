function output = controllerCartSpace(input)

%Proportial and Derivative gains (gain matrices)
global Kp_ Kd_;


%Desired and actual joint position in joint space
des_q = [input(1) input(2)]';
q = [input(5) input(6)]';


%Desired and actual joint position vectors  in joint space
des_dq = [input(3) input(4)]';
dq = [input(7) input(8)]';


%Actual foot position
Xf = forwardKinematics(q);


%Desired foot position
des_Xf = forwardKinematics(des_q);


%Leg Jacobian
J = hylJac(q);


%Control action
output = J' * ( Kp_ * (des_Xf - Xf) + Kd_ * J * (des_dq -dq) );

% output = pinv(output)*output*[5 5]';

end