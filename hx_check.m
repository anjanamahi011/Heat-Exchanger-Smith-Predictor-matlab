clear; clc; close all; rng(2);
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;
Gs = tf(1, [5 1]);                            % RTD sensor lag, 5 s

% Final design (same as hx_final2.m)
thetaE = theta + 5;
Kc1 = tau/(K*(2*thetaE));  Ti1 = min(tau, 8*thetaE);
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);

Kc2 = tau/(K*10);  Ti2 = tau;
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);
Gm0 = tf(K, [tau 1])*Gs;
Gm  = tf(K, [tau 1], 'InputDelay', theta)*Gs;
Ceff = feedback(C2, Gm0 - Gm);

fprintf('PI:  Kc = %.3f  Ti = %.1f s\n', Kc1, Ti1);
fprintf('PI+Smith:  Kc = %.3f  Ti = %.1f s\n\n', Kc2, Ti2);

cases = {'Nominal','Delay +30%','Delay -30%','Gain +30%','Gain -30%'};
kf = [1 1 1 1.3 0.7];  df = [1 1.3 0.7 1 1];

% Sensitivity peak Ms for each case
w = logspace(-3, 1, 3000);
fprintf('%-12s | Ms PI | Ms PI+Smith\n', 'Case');
for i = 1:5
    Gp = tf(1.5*kf(i), [50 1], 'InputDelay', 15*df(i));
    S1 = 1./(1 + squeeze(freqresp(C1*Gp*Gs,   w)));
    S2 = 1./(1 + squeeze(freqresp(Ceff*Gp*Gs, w)));
    fprintf('%-12s | %5.2f | %5.2f\n', cases{i}, max(abs(S1)), max(abs(S2)));
end

% Valve movement from RTD noise (nominal plant)
Gp = tf(1.5, [50 1], 'InputDelay', 15);
t  = (0:0.5:600)';
n  = 0.05*randn(size(t));
u1 = lsim(-feedback(C1,   Gp*Gs), n, t);
u2 = lsim(-feedback(Ceff, Gp*Gs), n, t);
fprintf('\nValve movement from noise (std, %% open): PI = %.3f   PI+Smith = %.3f\n', ...
        std(u1), std(u2));

% Figure 1: nominal setpoint and disturbance
t = (0:0.5:800)';
figure('Position',[100 100 1000 400]);
subplot(1,2,1);
plot(t, step(10*feedback(C1*Gp, Gs), t), t, step(10*feedback(Ceff*Gp, Gs), t));
grid on; xlim([0 300]); legend('PI','PI + Smith predictor');
title('Setpoint step (nominal)'); xlabel('Time (s)'); ylabel('Temperature rise (degC)');
subplot(1,2,2);
plot(t, step(5*feedback(Gp, C1*Gs), t), t, step(5*feedback(Gp, Ceff*Gs), t));
grid on; xlim([0 400]); legend('PI','PI + Smith predictor');
title('Load disturbance (nominal)'); xlabel('Time (s)'); ylabel('Deviation (degC)');
exportgraphics(gcf, 'fig_nominal.png', 'Resolution', 200);

% Figure 2: robustness cases
figure('Position',[100 100 1200 700]);
for i = 1:5
    Gp = tf(1.5*kf(i), [50 1], 'InputDelay', 15*df(i));
    subplot(2,3,i);
    plot(t, step(10*feedback(C1*Gp, Gs), t), t, step(10*feedback(Ceff*Gp, Gs), t));
    grid on; xlim([0 300]); title(cases{i}); xlabel('Time (s)'); ylabel('degC');
    if i == 1, legend('PI','PI + Smith','Location','southeast'); end
end
exportgraphics(gcf, 'fig_robustness.png', 'Resolution', 200);

disp('Saved fig_nominal.png and fig_robustness.png');