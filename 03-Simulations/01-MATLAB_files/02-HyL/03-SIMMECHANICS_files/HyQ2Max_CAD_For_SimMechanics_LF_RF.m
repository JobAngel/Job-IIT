%M-file for HyQ2Max LF and RF legs parameters from Marco Frigerio data
%A similar approach can be used to convert the parameters for LH and RH legs
%and HAA (Hip Abduction/Adduction)
%In SimMechanics World coordinates origin are at Upper Leg joint HFE
% +x - axis point to the right
% +y - axis points up
% +z - axis points out of the screen
%In the HyQ convention (see document HyQV1LegV2_StandardDefinitions_v2.pdf)
% +x - axis point to the right
% +y - axis points into the screen
% +z - axis points up
%in HyQ2Max the body axes conventions are the same as in HyQ
%In HyQ the body axes for the LF and RF axes are obtained by ZXZ Euler
%rotation of the SimMechanics World coordinate, -90 degrees about the z axis,
%then 180 degrees about the new x axis and then 0 degrees about the new z axis
%In this manner the body Inertia and Cog parameters can be given directly from
%the CAD data in the document HyQLegV2.5_InertiaParameters.pdf
%The Cog of each body is given in the Body coordinates
%The Inetia of each body is given about the Cog and in the body coordinates
%Points in body where joints connect in World coordinates
%rotations in the revolute joinst are in world coordinates about the - z axis
%so that the visualization of the leg coincides with the HyQ angle convention
%as given in the document HyQV1LegV2_StandardDefinitions_v2.pdf
%Gustavo A. Medrano-Cerda Feb 2016

clc
clear all;
format long

% Upper leg HFE, Hip Flexion/Extension (LF/RF)
U_LegM=4.95;  %Kg

U_LegCog_CAD=[0.144296 -0.002071 -0.005094]'

U_LegInertia_CAD=[0.010196 0.002320 0.000141
 0.002320 0.204183 -0.000034
 0.000141 -0.000034 0.207532]

% Lower leg KFE, Knee Flexion/Extension (LF/RF non compliant foot)
L_LegMb=1.406; %Kg

L_LegCogb_CAD=[0.084410 0.040548 0.000740]'

L_LegInertiab_CAD=[
0.005856  -0.007627 -0.000009
-0.007627  0.031471 -0.000013
-0.000009 -0.000013  0.034133]

%Check INERTIA with respect to ACS0 coordinate frame should equal
%(L_LegInertiab_CAD+L_LegMb*(L_LegCogb_CAD'*L_LegCogb_CAD*eye(3)-L_LegCogb_CAD*L_LegCogb_CAD'))*1e6  %written when L_LegCogb_CAD is column vector

%For SimMechanics
%Upper Leg coordinates
U_LegCS1=[0 0 0];     %in World Coordinates
U_LegCS2=[0.36 0 0];  %translated from U_LegCS1 and in body coordinates

%Lower Leg with non compliant foot
L_LegCS1b=U_LegCS2;      %translated from adjoining body (Upper Leg)
L_LegCS2b=[0.38009 0 0]; %translated from L_LegCS1b and in body coordinates
