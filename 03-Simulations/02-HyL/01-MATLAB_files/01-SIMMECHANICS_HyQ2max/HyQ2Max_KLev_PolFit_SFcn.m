function HyQ2Max_KLev_PolFit_SFcn(block)
% Level-2 M file S-Function HyQ2Max Knee Lever for LF/RF leg
% 5th order polynomial fit for parameters in YiFu's M file
% inpp=[320 -23 -41 5 56 56 34 314*10^-6 39.5]; 
% inpp=[mm mm mm mm mm mm mm mm^2 deg];
% inpp=[ax(1) ay(2) cx(3) cy(4) lra(5) lrb(6) lkb(7) piston_area(8) offest(9)], input parameters
% polynomial fit for: Lever(-ang), ang in radians, Lever in mm
%   Gustavo A. Medrano-Cerda Feb 2016
%    

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).DirectFeedthrough = true;
  
  %% Set block sample time to inherited
  block.SampleTimes = [-1 0];
  
  %% Run accelerator on TLC
  block.SetAccelRunOnTLC(true);
  
  %% Register methods
  block.RegBlockMethod('Outputs',                 @Output);  
  
%endfunction

function Output(block)

KneeAng=block.InputPort(1).Data;   %in rad
mu=-1.4835;
sig=0.84014;
temp=(KneeAng-mu)/sig;  %normalized angle

%Knee joint
Lp=[-0.2215   -0.1788    3.0013   -2.7578  -12.9684   26.7717]; 

KneeLever=(Lp(1)*(temp)^5 + Lp(2)*(temp)^4 + Lp(3)*(temp)^3 + Lp(4)*(temp)^2 + Lp(5)*(temp) + Lp(6));

block.OutputPort(1).Data = KneeLever/1000;  %divide by 1000 to convert to m
  
%endfunction

