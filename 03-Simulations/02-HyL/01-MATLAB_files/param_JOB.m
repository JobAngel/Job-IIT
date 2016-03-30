% Data obtained from:
% ------------------------------------------------------------
% HyQ-Robot: Standard Definition for Joint Angles
% and Kinematic Parameters of the Legs and Torso
% 
% Project: HyQ-Robot V1.0 with Leg V2.0 
% Genoa, 9th July 2010
%-------------------------------------------------------------
% Code made by:
% JOB A. LEDEZMA, 2016
%-------------------------------------------------------------


% ________________________________________
%
%         HYL DESCRIPTION PARAMETERS
%_________________________________________

% ---- Leg data ------

m1 = 1.772;              % Kg - Upper leg mass
m2 = 0.808;              % Kg - Lower leg mass
M = 3.982;                % Kg - Base mass 
P_01 = 0.08;             % m - distance P0-P1
P_12 = 0.350;           % m - distance P1-P2
P_23 = 0.330;           % m - distance P2-P3
FR = 0.02;                 % m - foot radius
P1m1 = 0.164;           % m - distance P1-Pm1 
P2m2 = 0.122;           % m - distance P2-Pm2
I1zz = 0.0713;           % kg·m2 - Inertia upper leg
I2zz = 0.0218;           % kg·m2 - Inertia lower leg
em1 = deg2rad(7.9);  % rad 
em2 = deg2rad(0);     % rad  
g  = 9.810;                 % m/s2 - gravity

% ----- Lever Arm Parameters -----

% Hip joint
a1   = 0.3219;             % m
b1   = 0.045;               % m
e11 = deg2rad(6.24); % rad
e12 = deg2rad(0);      % rad

% Knee joint
a2 = 0.3218;               % m
b2 = 0.045;                 % m
e21 = deg2rad(8.04);  % rad
e22 = deg2rad(6);       % rad

% ----- Limit Angles --------

% Initial conditions
q0_0 = 0.75;                           % m - height of the floating base
q1_0 = deg2rad(45);         % rad - HFE angle
q2_0 = deg2rad(-90);        % rad - KFE angle

% Max angle limits
q1Max = deg2rad(70);       % rad
q2Max = deg2rad(-20);      % rad

% Min angle limits
q1Min = deg2rad(-50);        % rad
q2Min = deg2rad(-140);      % rad
q0Min = P_01+P_12*cos(q1Min)+P_23*cos(q1Min+q2Min)+FR;     % m

% ________________________________________________________
%
%                    HYDRAULIC PARAMETERS
% ________________________________________________________

% ------ Power unit --------

ps  = 160e5;                            % Pa - Supply pressure
pt  = 10e5;                              % Pa - Tank pressure
dpt = ps - pt;                           % Pa - Delta pressure
beta = 1.4e9;                          % Pa- Effective bulk modulus 

% Initial condition for integrators

xo = 0;
pAo = 0;%ps/2;
pBo = 0;%ps/2;

%----- Cylinder - HOERBIGER LB6 16 10 0080 4M ----------

Dp = 0.016;                             % m - Piston diameter
Dr = 0.010;                              % m - Rod diameter
L  = 0.080;                               % m- Stroke
Aa = pi*(Dp^2)/4;                   % m^2 - Area A of the piston
Ab = pi*(Dp^2-Dr^2)/4;         % m^2 - Area B of the piston
r  = Aa/Ab;                              % Relation between areas
fv = 700;                                 %  Ns/m - Viscous friction
leak =1.7*10^-13;                 % m3/(s.Pa) - Cylinder leakage
Mp = ((pi*Dr^2/4)*L + Aa*0.15)*7860;    % Kg - Piston mass (approximated)
Lcyl = 0.280;                           % m - Total length of the cylinder
 
%-----  Valve Parameters - MOOG E024 - 177LA ----------
 
In   = 10e-3;                               % I - Nominal control current
Qn   = 7.5 / 60e3;                 % m^3/s - Nominal flow rate
dptn = 70e5;                         % Pa - Valve pressure drop
Qlk   = 0.375 / 60e3;               % m^3/s - Leakage flow
Plk    = 70e5;                       % Pa - Nominal supply pressure for leakage test
wn    = 2*100*pi;                    % rad/s - Valve's natural frequency
E       = 0.7;                           % Valve's damping coefficient  
Kv     = Qn/sqrt(dptn);          % m^3/(s*sqrt(Pa)) - Total flow coefficient
Kva    = Kv*sqrt(2);               % m^3/(s*sqrt(Pa)) - Partial flow coefficient A
Kvb    = Kv*sqrt(2);               % m^3/(s*sqrt(Pa)) - Partial flow coefficient B
Kvlk   = Qlk/sqrt(2*Plk);         % m^3/(s*sqrt(Pa)) - Total leakage flow coefficient
KvlkA  = Kvlk;                         % m^3/(s*sqrt(Pa)) -  Partial leakage flow coefficient A
KvlkB  = Kvlk;                         % m^3/(s*sqrt(Pa)) - Partial leakage flow coefficient B
up_dz = 0;                             % V - Upper limit of dead zone
lw_dz = 0;                              % V - Lower limit of dead zone

%------------   Manifold  -----------
% Review this part with the manifold's CAD design

v1a = (pi*(0.0035^2)/4)*0.07255;
v2a = (pi*(0.006^2)/4)*(0.090-0.012);
v1b = (pi*(0.0035^2)/4)*0.04491;
v2b = (pi*(0.006^2)/4)*(0.019+0.009);
v3b = (pi*(0.006^2)/4)*(0.007+0.009);
V0A = v1a + v2a;                                            % Dead volume coupled to chamber A
V0B = v1b + v2b + v3b;                                  % Dead volume coupled to chamber B

%-------  Load cell - BURSTER 8417-6005 ----------

xs = 20e-6;                            % m - Maximum deflection @ full scale
Fs = 5000;                             % N - Maximum force
Ks = Fs / xs ;                         % N·m - Load cell stiffness

noise = 0;                            % N - noise due nonlinearities, hysteresis, non repeatibility

%------ Accumulator - FOX-ITALY HST 0,04 ----------

n = 1.4;                    % Polytropic constant
Tamb = 298;
P0 = 100e5;
Pmax_acc = 320e5;
V0 = 0.04e-3;               % m3 - Initial volume of the accumulator 
h = 10;                     % W/(m^2·K)- Convection heat transfer coefficient
Aw = 0.01;                  % m^2 - Thermal exchange area
dacc = 10e-3;                % m - Diameter of the inlet orifice 
Adacc = pi*dacc^2/4;        % m^2 - Area of the inlet orifice of the accumulator
rho = 800;                  % Kg/m^3 - Fluid density
mf = 0;
cf = 0;

% TESTS

Kp = 1e-4;
Ki = 1e-4;
Kd = 1e-4;
N = 100;