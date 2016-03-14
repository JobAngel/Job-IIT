% FC1D test bench data
% --------------------
% All of the data include in this file was obtained from manufacturer's
% catalog.
% 
% The present file allows the selection of the High Volumetris Expansion
% Hoses to be used in the FC1D test bench of the Hydraulic Quadruped Robot
% Project.


close all
clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                  HYDRAULIC POWER UNIT                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ps  = 160e5;                            % (Pa)    Supply pressure
pt  = 1e5;                              % (Pa)    Tank pressure
dpt = ps - pt;                          % (Pa)    Delta pressure
beta = 1.35e5;%1.4e9;                           % (Pa)    Effective bulk modulus 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Cylinder Parameters                    %
%               HOERBIGER LB6 16 10 0080 4M                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Dp = 0.016;                             % (m)     Piston diameter
Dr = 0.010;                             % (m)     Rod diameter
L  = 0.080;                             % (m)     Stroke
Aa = pi*(Dp^2)/4;                       % (m^2)   Area of the cylinder side A
Ab = pi*(Dp^2-Dr^2)/4;                  % (m^2)   Area of the cylinder side B
r  = Aa/Ab;                             % (-)     Relation between areas
fv = 700;                               % (Ns/m)  Viscous friction
% leak = 1.7*10^-13;                    % (m3/(s.Pa)) Cylinder leakage
Mp = ((pi*Dr^2/4)*L + Aa*0.15)*7860;    % (Kg)    Piston mass (approximated)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Valve Parameters                       %
%                   MOOG E024 - 177LA                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Un     = 10;                           % (N) Nominal tension
Qn     = 5 / 60e3;                     % (m^3/s) Nominal flow rate
dptn   = 70e5;                         % (Pa) Valve pressure drop

Qlk    = 0.28 / 60e3;                  % (m^3/s) Leakage flow
Plk   = 210e5;                         % (Pa) Nominal supply pressure for leakage test

wn     = 2*32*pi;                      % (rad/s) Valve's natural frequency
E      = 0.9;                          % (-) Valve's damping coefficient  

Kv     = Qn/sqrt(dptn);                % (m^4/s*N^(1/2)) Total flow coefficient

Kva    = Kv*sqrt(2);                   % (m^4/s*N^(1/2)) Partial flow coefficient A
Kvb    = Kv*sqrt(2);                   % (m^4/s*N^(1/2)) Partial flow coefficient B

Kvlk   = Qlk/sqrt(2*Plk);              % (m^4/s*N^(1/2)) Total leakage flow coefficient
KvlkA  = Kvlk;                         % (m^4/s*N^(1/2)) Partial leakage flow coefficient A
KvlkB  = Kvlk;                         % (m^4/s*N^(1/2)) Partial leakage flow coefficient B

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         Manifold                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

v1a = (pi*(0.0035^2)/4)*0.07255;
v2a = (pi*(0.006^2)/4)*(0.090-0.012);
V0A = v1a + v2a;

v1b = (pi*(0.0035^2)/4)*0.04491;
v2b = (pi*(0.006^2)/4)*(0.019+0.009);
v3b = (pi*(0.006^2)/4)*(0.007+0.009);
V0B = v1b + v2b + v3b;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      Load cell                           %
%                   BURSTER 8417-6005                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xs = 6e-6;                             % (m) Maximum deflection @ full scale
Fs = 5000;                             % (N) Maximum force
Ks = Fs / xs ;                         % (N·m) Load cell stiffness

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Parameters of viscous friction coefficient         %
%          Gomes(2003): variable friction model            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Polyp  = [0.4040e4   -1.5314e4    2.2225e4   -1.5405e4    0.5502e4   -0.0647e4    0.0083e4];
 
Polyn  = [-0.4083e4  -1.5245e4   -2.2067e4   -1.5466e4   -0.5594e4   -0.0664e4   -0.0087e4];                      

Fsp    = 101.65;                           
Fsn    = -99.29;                           

dxlimp = 0.0035;                           
dxlimn = -0.0035;                          

dx0p   = 0.0035*0.95;                      
dx0n   = -0.0035*0.95;                     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             Parameters for SIMULATION                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kent = 1e9;

% Accumulator

% acc = 0;   
% n = 1.4;
% V0 = 0.32e-3;
% P0 = 35e5 + 1e5;
acc = 1;   
n = 1.4;
V0 = 0.16e-3;
P0 = 120e5 + 1e5;

% CONTROLADORES
% -------------
control = 3;

switch control
    case 1
        % Control I: tss = 1 seg; beta = 1.4e9; Ke = 6.5e5;
        %            Força sobre parede
        numC = [0.007381 0.7753 7.326];
        denC = [1 192.8 0];
        numF = 46800;
        denF = [1 253 6720 46800];
    case 2    % Controlador 1_final + beta da mangueira + sem Ke nem Xe
        numC = [1.818 33.64 150];
        denC = [1 600 0];
        numF = 150;
        denF = [1 25 150];
    case 3    % Controlador 2_final + beta da mangueira + sem Ke nem Xe
        numC = [0.004938 0.1556 0.4];
        denC = [1 0 0];
        numF = 121;
        denF = [1 19.8 121];

    case 4
        % Control III: tss = 0.5 seg; beta = 1.4e9; 
        %             Assumindo sistemas separados sem Ke e com xe
        numC = [0.00751 0.2373  6.551 101.3 367.7];
        denC = [1 9.6 576 0 0];
        numF =  182.3;
        denF = [1 27 182.3];

end

% Dados do controle PID mangueira
Kp_hose = 0.021; 
Ki_hose = 1e-4;
Kd_hose = 0.00192;
N_hose = 6.315;

% Dados do controle PID tubo
Kp_pipe = 0.00001; 
Ki_pipe = 0;
Kd_pipe = 0;
N_pipe = 100;

Cex = 0;

% ruido de mediçao
ruido = 5;

% Seletores
% ---------
tipo_controlador = 1;  % 1: PID
                       % 2: QFT

sel_atrito_hid = 2;    % 1: Atrito variável
                       % 2: Atrito viscoso constante

sel_ruido = 1;         % 1: Sem ruído de mediçao
                       % 2: Com ruido de mediçao
                       
tipo_ref = 1;          % 1: STEP
                       % 2: TREM DE PULSOS
                       % 3: SENOIDE
                       % 4: TRAJETORIA
                    
seletor_pert = 0;      % 0: off (entorno estático)
                       % 1: on (entorno dinamico)
                    
tipo_pert = 1;         % 1: STEP
                       % 2: TREM DE PULSOS
                       % 3: SENOIDE
                    
Kmola = 6e5;
Ke = ( Kent^-1 + Ks^-1)^-1;
Kcc = Ks;%( Kent^-1 + Kmola^-1+ Ks^-1)^-1;

% Condiçoes iniciais
% ------------------
pAo = ps/2;
pBo = ps/2;
xo  = 0;

% Tipos de referencia
% -------------------

% STEP 
ref_step_tempo = 1;
ref_step_degrau = 500;

% TREM DE PULSOS
ref_trem_amp = 5000;
ref_trem_periodo = 4;
ref_trem_ciclo = 50;

% SENOIDE
ref_sen_amp = 5000;
ref_sen_bias = 0;
ref_sen_freq = 1;
temp_pert = 0;
switch tipo_ref
    case 1 % STEP
        tempo_sim = 5;
        temp_pert = 3;
    case 2 % TREM DE PULSOS
        tempo_sim = 10;
        temp_pert = 5;
    case 3 % SENOIDE
        tempo_sim = 10;
        temp_pert = 5;
    case 4 % TRAJETORIA
        tempo_sim = 30;
        temp_pert = 20;
end

% Seletor de tipo de perturbaçao
% ------------------------------

% STEP
pert_step_tempo = 3;
pert_step_degrau = 100e-3;

% TREM DE PULSOS
pert_trem_amp = 50e-3;
pert_trem_periodo = 2;
pert_trem_ciclo = 50;

% SENOIDE
pert_sen_amp = 1e-3;
pert_sen_bias = 0;
pert_sen_freq = 6;

% Definiçao do SAMPLE TIME
% ------------------------
% Escolher -1 para inabilitá-lo
sample_time = -1;
decimation = 1;

% sample_time = 1;
% decimation = 200;


% sim('teste_modelo_journal');
sim('FC1D');

tf = length(saida.time);
t0 = 1;
tempo = saida.time(t0:tf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
% força
subplot(2,1,1)
stairs(tempo,saida.signals.values(t0:tf,1),'-r')
hold on

plot(tempo,saida.signals.values(t0:tf,8),'-.k','LineWidth',1); 
plot(tempo,saida.signals.values(t0:tf,11),':b','LineWidth',1);
% plot(tempo,saida.signals.values(t0:tf,5),'g','LineWidth',1);
legend('Reference','PID-pipe','PID-hose')
xlabel('Time (s)');ylabel('Force (N)');
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% figure(3)
% % controle 
% 
subplot(2,1,2);
plot(tempo,saida.signals.values(t0:tf,9),'-.k','LineWidth',1);hold on
plot(tempo,saida.signals.values(t0:tf,12),':b','LineWidth',1);hold on
% plot(tempo,saida.signals.values(t0:tf,6),'g','LineWidth',1);

xlabel('Time (s)');ylabel('Control voltage (V)');
grid on
% hold on
% 
% % % erro
% subplot(3,1,3)
% plot(tempo,saida.signals.values(t0:tf,10),'k',tempo,saida.signals.values(t0:tf,4),'b',tempo,saida.signals.values(t0:tf,7),'g');%,tempo,saida.signals.values(t0:tf,10),'k');
% xlabel('Tempo [s]');ylabel('erro [N]');
% hold on 
% grid on

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% % figure(2)
% % % posiçao do entorno 
% subplot(2,2,3);
% plot(tempo,saida.signals.values(t0:tf,11).*1000,'m');
% xlabel('Tempo [s]');ylabel('Posição entorno [mm]');
% grid on
% hold on

% % % posiçao da haste
% subplot(2,1,2)
% plot(tempo,saida.signals.values(t0:tf,12).*1000,'b',tempo,saida.signals.values(t0:tf,13).*1000,'g',tempo,saida.signals.values(t0:tf,14).*1000,'k');%,tempo,saida.signals.values(t0:tf,14).*1000,'k');
% legend('QFT-pipe','QFT-hose','PID-pipe')
% xlabel('Tempo [s]');ylabel('Posição haste [mm]');
% hold on 
% grid on

% %%
% saveas(gcf,'QFT_Fe_out','fig')
% export_fig QFT_Fe_out -eps -painters -CMYK -transparent -depsc -nocrop


%% Calculo do RMSE

% RMSE_QFTp = sqrt(mean(saida.signals.values(t0:tf,4).^2))
RMSE_QFTh = sqrt(mean(saida.signals.values(t0:tf,7).^2))
% RMSE_PIDp = sqrt(mean(saida.signals.values(t0:tf,10).^2))
RMSE_PIDh = sqrt(mean(saida.signals.values(t0:tf,13).^2))

% Calculo do RMS controle

% RMS_ctrl_QFTp = sqrt(mean(saida.signals.values(t0:tf,3).^2))
RMS_ctrl_QFTh = sqrt(mean(saida.signals.values(t0:tf,6).^2))
% RMS_ctrl_PIDp = sqrt(mean(saida.signals.values(t0:tf,9).^2))
RMS_ctrl_PIDh = sqrt(mean(saida.signals.values(t0:tf,12).^2))
