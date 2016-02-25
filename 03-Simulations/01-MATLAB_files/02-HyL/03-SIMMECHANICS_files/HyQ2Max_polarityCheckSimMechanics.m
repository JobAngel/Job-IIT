
close all;

figure;

p1 = subplot(4,1,1),plot(time, LF_HFE_th,'LineWidth',2);
title('Polarity Checks for HFE Joint');
ylabel('Joint Pos [rad]');
grid on;
p2 = subplot(4,1,2),plot(time, LF_HFE_load,'LineWidth',2);
ylabel('Joint Load [Nm]');
grid on;
grid on;
linkaxes([p1 p2],'x');
xlim([3 4]);


figure;

p5 = subplot(4,1,1),plot(time, LF_KFE_th,'LineWidth',2);
title('Polarity Checks for KFE Joint');
ylabel('Joint Pos [rad]');
grid on;
p6 = subplot(4,1,2),plot(time, LF_KFE_hst_xp,'LineWidth',2);
ylabel('Piston Pos [m]');
grid on;
p7 = subplot(4,1,3),plot(time, LF_KFE_hst_force.*LF_KFE_lever,'LineWidth',2);
ylabel('Joint Load [Nm] (force*lever)');
grid on;
p8 = subplot(4,1,4),plot(time, LF_KFE_hst_force,'LineWidth',2);
ylabel('Cyl Force [N]');
xlabel('Time [s]');
grid on;
linkaxes([p5 p6 p7 p8],'x');
xlim([3 4]);