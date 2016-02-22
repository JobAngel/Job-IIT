function jac = hylJac(q)

l0 = 0.09; %Base to HFE distance
l1 = 0.35; %HFE to KFE distance
l2 = 0.33; %KFE to foot distance
l3 = 0.02; %Foot radius (not used)

J11 = - l1 * cos( q(1) ) - l2 * cos( q(1) + q(2) );
J12 = - l2 * cos (q(1) + q(2));
J21 = l1 * sin( q(1)) + l2 * sin( q(1) + q(2) );
J22 = l2 * sin( q(1) + q(2) );

jac = [J11 J12; J21 J22];

end