function X = forwardKinemtics(q)

%Forward Kinematics (from Semini PhD Thesis)

l0 = 0.09; %Base to HFE distance
l1 = 0.35; %HFE to KFE distance
l2 = 0.33; %KFE to foot distance
l3 = 0.02; %Foot radius (not used)

x = - l1 * sin( q(1) ) - l2 * sin( q(1) + q(2) );
%z = - l0 - l1 * cos( q(1) ) - l2 * cos( q(1) + q(2) ) - l3; %Considering
%foot radius
z = - l0 - l1 * cos( q(1) ) - l2 * cos( q(1) + q(2) );

X = [x z]';

end