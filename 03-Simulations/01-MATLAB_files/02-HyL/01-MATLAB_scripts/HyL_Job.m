function hylmodel = HyL_Job()

% HyL model parameters: 
%               provided by Marco Frigerio.
% The model works with Spatial Vector and Rigid-Body Dynamics Software: 
%               Spatial V2.0 by Roy Featherstone.
% .....................................................................
% In order to see the virtual showmotion you must download and install
% the Spatial V2.0 software: http://royfeatherstone.org/spatial/v2/

% ** The parameters into the Simulink model are automatically charged, 
%    you only need to run the simulator.
%......................................................................

% File made by:             Job Angel Ledezma
% Under the supervision of: Victor Barasuol
%                           Claudio Semini

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

P_01 = 0.08;			% distance between P0 and P1
P_12 = 0.35;			% distance between P1 and P2
P_23 = 0.33;			% distance between P2 and P3
R_foot = 0.02;          % Foot radius
        
% INERTIA DATA - LINK COORDINATES
% (coordinate system of the front left leg)
% Marco Frigerio's LEG data

trunk_mass = 3.982;
trunk_CoM = [-0.08315; 0.06849; -0.00654];
trunk_Icm = [ 0.037131,	-0.028543, 9.7E-4;
 -0.028543, 0.087473 , -0.003392;
 9.7E-4, -0.003392, 0.112496];

upperleg_mass = 2.06218;
upperleg_CoM = [0.14925; -0.02996; -5.0E-5];
upperleg_Icm = [4.64e-3, 0.00718, 1.0E-5;
 0.00718, 0.07216 ,	1.0E-5;
 1.0E-5, 1.0E-5, 0.07463];

lowerleg_mass = 0.80669;
lowerleg_CoM = [0.12185; 5.8E-4; -1.2E-4];
lowerleg_Icm = [ 4.2E-4, 0,	0; 0, 0.02202, 0; 0, 0,	0.02183];

% Marco's LEG description

hylmodel.Xtree{1} = eye(6);
hylmodel.Xtree{2} = rotz(pi/2) * rotx(-pi/2) * xlt([0 0.13 -P_01]);
hylmodel.Xtree{3} = xlt([P_12 0 0]);

hylmodel.I{1} = mcI(trunk_mass, trunk_CoM, trunk_Icm );
hylmodel.I{2} = mcI(upperleg_mass, upperleg_CoM, upperleg_Icm );
hylmodel.I{3} = mcI(lowerleg_mass, lowerleg_CoM, lowerleg_Icm );


% APPEARANCE
% Bodies plotting
% hylmodel.appearance.body{1} = ...
%   { 'box', [0,-0.015,-0.02; d_01,0.015,0.02], ...
%     'cyl', [0,0,-0.025; 0,0,0.025], 0.02, ...
%     'cyl', [d_01,-0.02,0; d_01,0.02,0], 0.025 };

% This code plots the axes
% X: red line
% Y: green line
% Z: blue line

plot_axes = {'colour',[1 0 0 ],'cyl',[0 0 0; 0.2 0 0], 0.002,...
    'colour',[0 1 0 ],'cyl',[0 0 0; 0 0.2 0], 0.002,...
    'colour',[0 0 1 ],'cyl',[0 0 0; 0 0 0.2], 0.002};  

hylmodel.appearance.body{1} = { plot_axes };
hylmodel.appearance.body{2} = { 'cyl', [0,0,0; P_12,0,0], 0.015, plot_axes };
hylmodel.appearance.body{3} = { 'cyl', [0,0,0; P_23,0,0], 0.012, plot_axes, 'sphere', [P_23,0,0], R_foot };

% Floor plotting
hylmodel.appearance.base = { 'tiles', [-0.5 0.5; -0.37 0.63; 0 0], 0.25, plot_axes};

% Camera
hylmodel.camera.body = 1;
hylmodel.camera.direction = [0 1 0.5];
hylmodel.camera.zoom = 0.5;
 
% GROUND CONTACT
hylmodel.gc.body = 3; 
hylmodel.gc.point = [P_23+R_foot;0;0];


