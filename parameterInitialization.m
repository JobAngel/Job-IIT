

%-----------------------------------------------------
%Controller parameters
%-----------------------------------------------------

%Proportional and derivative gains in joint space
global Kp Kd;

Kp = 200 * eye(2,2);
Kd = 6 * eye(2,2);

%Proportional and derivative gains in Cartesian space
global Kp_ Kd_;

Kp_ = 5000 * eye(2,2);
Kd_ = 50 * eye(2,2);


%-----------------------------------------------------
%Leg parameters
%-----------------------------------------------------

%global l1 l2;

%l0 = 0.09;         % Base to HFE joint
%l1 = 0.35;			% upper limb length
%l2 = 0.33;			% lower limb length
%l3 = 0.02;         % foot radius
