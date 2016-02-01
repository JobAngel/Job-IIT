function hylmodel = HyL_Job()
% This file contains:
% Marco's HyL model and Roy's HyL (extracted from his HyQ model)

% In order to see the virtual showmotion you will require the
% Roy's software: http://royfeatherstone.org/spatial/v2/

% After installing the soft in Matlab, you need to put this code 
% into Matlab's workspace: showmotion(HyL_Job,qout.Time,qout.data)
% to see the simulation.

% ** The parameters into the Simulink model are automatically charged, 
%    you only need to run the simulator.


hylmodel.robotname = 'HyL_Job';

hylmodel.NB = 3;
hylmodel.parent = zeros(1,3);

hylmodel.parent(1) = 0;
hylmodel.parent(2) = 1;
hylmodel.parent(3) = 2;

hylmodel.jtype{1} =  'P' ;
hylmodel.jtype{2} =  'R' ;
hylmodel.jtype{3} =  'R' ;

% LENGTH DATA

d_01 = 0.08;			% distance between P0 and P1
d_12 = 0.35;			% distance between P1 and P2
d_23 = 0.33;			% distance between P2 and P3
R_foot = 0.02;          % Foot radius

selector = 2;           % select the HyL model
                        % 1: Marco's model
                        % 2: Roy's model

switch (selector)
    case 1
        
% INERTIA DATA - LINK COORDINATES
% (coordinate system of the front left leg)

% Marco Frigerio's LEG data

hip_mass = 3.982;
hip_CoM = [-0.08315; 0.06849; -0.00654];
hip_Icm = [ 0.037131,	-(0.028543),	-(-9.7E-4);
 -(0.028543),	  0.087473 ,	-(0.003392);
 -(-9.7E-4),	-(0.003392),	  0.112496];

upperleg_mass = 2.06218;
upperleg_CoM = [0.14925; -0.02996; -5.0E-5];
upperleg_Icm = [ 0.00464,	-(-0.00718),	-(-1.0E-5);
 -(-0.00718),	  0.07216 ,	-(-1.0E-5);
 -(-1.0E-5),	-(-1.0E-5),	  0.07463];

lowerleg_mass = 0.80669;
lowerleg_CoM = [0.12185; 5.8E-4; -1.2E-4];
lowerleg_Icm = [ 4.2E-4,	-(0.0),	-(0.0);
 -(0.0),	  0.02202 ,	-(0.0);
 -(0.0),	-(0.0),	  0.02183];

% Marco's LEG description

hylmodel.Xtree{1} = eye(6);
% I change this part to obtain the interaction with the ground,
% the real code is on the final of the line
hylmodel.Xtree{2} = rotx(-pi/2) * rotz(pi/2) * xlt([0 0.13 -d_01]);%rotz(pi/2) * rotx(-pi/2) * xlt([0 0.13 -d_01]);
hylmodel.Xtree{3} = xlt([d_12 0 0]);

hylmodel.I{1} = mcI(hip_mass, hip_CoM, hip_Icm );
hylmodel.I{2} = mcI(upperleg_mass, upperleg_CoM, upperleg_Icm );
hylmodel.I{3} = mcI(lowerleg_mass, lowerleg_CoM, lowerleg_Icm );

    case 2
% Roy's LEG data
hip_mass = 2.93;
hip_CoM = [0.04263; 0; 0.16931];
hip_Icm = ...
    [ 0.050713986, -3.6e-5,      -0.001586182; ...
     -3.6e-5,       0.054855242, -5.1e-5; ...
     -0.001586182, -5.1e-5,       0.0057082623 ];

upperleg_mass = 2.638;
upperleg_CoM = [-0.08315; 0.06849; -0.00654];
upperleg_Icm = ...
  [ 0.0036772534, -0.0030203676, 1.02e-4; ...
   -0.0030203676,  0.027193923,  2.1e-5; ...
    1.02e-4,       2.1e-5,       0.028111171 ];

lowerleg_mass = 0.881;
lowerleg_CoM = [0.1254; 0.0005; -0.0001];
lowerleg_Icm = ...
  [ 4.6777097e-4,  5.5238703e-5, -1.104774e-5; ...
    5.5238703e-5,  0.012555124,  -4.405e-8; ...
   -1.104774e-5,  -4.405e-8,      0.012326912 ];

% Roy's LEG description

hylmodel.Xtree{1} = eye(6);
% I change this part to obtain the interaction with the ground,
% the real code is on the final of the line
hylmodel.Xtree{2} = rotx(-pi/2) * rotz(pi/2) * xlt([d_01,0,0]); %rotx(pi/2) * xlt([d_01,0,0]);
hylmodel.Xtree{3} = xlt([d_12,0,0]);

hylmodel.I{1} = mcI(hip_mass, hip_CoM, hip_Icm );
hylmodel.I{2} = mcI(upperleg_mass, upperleg_CoM, upperleg_Icm );
hylmodel.I{3} = mcI(lowerleg_mass, lowerleg_CoM, lowerleg_Icm );

end

% APPEARANCE
% Floor plotting
hylmodel.appearance.base = { 'tiles', [-0.5 0.5; -1 1; 0 0], 0.25};

% Bodies plotting
% hylmodel.appearance.body{1} = ...
%   { 'box', [0,-0.015,-0.02; d_01,0.015,0.02], ...
%     'cyl', [0,0,-0.025; 0,0,0.025], 0.02, ...
%     'cyl', [d_01,-0.02,0; d_01,0.02,0], 0.025 };

hylmodel.appearance.body{2} = { 'cyl', [0,0,0; d_12,0,0], 0.015 };
hylmodel.appearance.body{3} = { 'cyl', [0,0,0; d_23,0,0], 0.012, 'sphere', [d_23,0,0], R_foot };

% GROUND CONTACT
hylmodel.gc.body = 3; 
hylmodel.gc.point = [d_23+R_foot;0;0];


