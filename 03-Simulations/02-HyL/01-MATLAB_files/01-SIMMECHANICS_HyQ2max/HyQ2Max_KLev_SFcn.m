function HyQ2Max_KLev_SFcn(block)
% Level-2 M file S-Function HyQ2Max Knee Lever for LF/RF leg
% inpp=[mm mm mm mm mm mm mm mm^2 deg];
% inpp=[ax(1) ay(2) cx(3) cy(4) lra(5) lrb(6) lkb(7) piston_area(8) offest(9)], input parameters
% KneeAng in radians, Lever in mm
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
inpp=[320  23   -41 -5   56 56 34   314*10^-6  -39.5]; %for LF/RF legs, negative joint angles

% inpp=[ax(1) ay(2) cx(3) cy(4) lra(5) lrb(6) lkb(7) piston_area(8) offest(9)], input parameters
ax=inpp(1);  %x y of point A,
ay=inpp(2);

cx=inpp(3);  %x y of point C, fixed pivot of cylinder
cy=inpp(4);

lra=inpp(5); %length of linkages AR BR KB
lrb=inpp(6);
lkb=inpp(7); 

sa=inpp(8); %cylinder area 
offest=inpp(9);
% constant
kx=360; % x of knee, knee=(360mm, 0)
ky=0;
% offest=39.5;   %unit:deg, offest from leg to the last link of 4 bar linkage

joint_angle=KneeAng/pi*180;
i=1;

    q=joint_angle - offest;    %% range of linkage 4 (4 linkage is KB GAMC Feb 2016)
    fai=q/180*pi;
    bxx= lkb*cos(fai)+kx;
    byy= lkb*sin(fai)+ky;

    bx(i)=bxx;
    by(i)=byy;

    % pr=solve('(x-bx)^2+(y-by)^2==lrb^2','(x-ax)^2+(y-ay)^2==lra^2');
    prx(1)=(ax*ay^2 - ax*bxx^2 - ax^2*bxx + ax*byy^2 + ay^2*bxx + bxx*byy^2 - ax*lra^2 + ax*lrb^2 + bxx*lra^2 - bxx*lrb^2 + ay*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) - byy*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + ax^3 + bxx^3 - 2*ax*ay*byy - 2*ay*bxx*byy)/(2*(ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2));
    prx(2)=(ax*ay^2 - ax*bxx^2 - ax^2*bxx + ax*byy^2 + ay^2*bxx + bxx*byy^2 - ax*lra^2 + ax*lrb^2 + bxx*lra^2 - bxx*lrb^2 - ay*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + byy*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + ax^3 + bxx^3 - 2*ax*ay*byy - 2*ay*bxx*byy)/(2*(ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2));
    pry(1)=(ax^2*ay + ay*bxx^2 + ax^2*byy - ay*byy^2 - ay^2*byy + bxx^2*byy - ay*lra^2 + ay*lrb^2 + byy*lra^2 - byy*lrb^2 - ax*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + bxx*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + ay^3 + byy^3 - 2*ax*ay*bxx - 2*ax*bxx*byy)/(2*(ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2));
    pry(2)=(ax^2*ay + ay*bxx^2 + ax^2*byy - ay*byy^2 - ay^2*byy + bxx^2*byy - ay*lra^2 + ay*lrb^2 + byy*lra^2 - byy*lrb^2 + ax*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) - bxx*((ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2 - lra^2 + 2*lra*lrb - lrb^2)*(- ax^2 + 2*ax*bxx - ay^2 + 2*ay*byy - bxx^2 - byy^2 + lra^2 + 2*lra*lrb + lrb^2))^(1/2) + ay^3 + byy^3 - 2*ax*ay*bxx - 2*ax*bxx*byy)/(2*(ax^2 - 2*ax*bxx + ay^2 - 2*ay*byy + bxx^2 + byy^2));

    %% select the right solution

    if pry(1)<0                      %GAMC Feb 2016
        rx(i)=prx(1);
        ry(i)=pry(1);
    else
        rx(i)=prx(2);
        ry(i)=pry(2);
    end
    %%
    %     rx(i)=prx(2);
    %     ry(i)=pry(2);
    PRX(i,:)=[prx(1) prx(2)];
    PRY(i,:)=[pry(1) pry(2)];

    lcr(i)=((rx(i)-cx)^2+(ry(i)-cy)^2)^0.5;  %cylinder length, eye to eye lenght
    ang(i)=joint_angle;  %output joint angle, 2deg to 168 deg

    VCR(i,1:2) = [rx(i)-cx ry(i)-cy]; %vector from point C to point R
    VAR(i,1:2) = [rx(i)-ax ry(i)-ay]; %vector from point A to point R
    alpha(i)=acos(dot(VCR(i,:),VAR(i,:))/(lcr(i)*lra));  % include angle between cr and ar

    VBR(i,1:2)=[rx(i)-bxx    ry(i)-byy];
    beta(i)=acos(dot(VAR(i,:),VBR(i,:))/(lra*lrb));

    VKB(i,1:2)=[bxx-kx     byy-ky];
    gamma(i)=acos(dot(VKB(i,:),VBR(i,:))/(lkb*lrb));

    KneeLever(i)=lkb*sin(alpha(i))*sin(gamma(i))/sin(beta(i)); %unit mm

block.OutputPort(1).Data = KneeLever(i)/1000;  %divide by 1000 to convert to m
  
%endfunction

