clear; clc; close all; rng(2);
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;

Gp = tf(1.5, [50 1], 'InputDelay', 15);
Gs = tf(1, [5 1]);                 % RTD sensor lag, 5 s (assumed value)

% Controller 1: plain PI
Kc1 = tau/(K*(2*theta));  Ti1 = min(tau, 8*theta);
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);

% Controller 2: PI + Smith predictor
Kc2 = tau/(K*10);  Ti2 = tau;
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);
Gm0 = tf(K, [tau 1]);
Gm  = tf(K, [tau 1], 'InputDelay', theta);
Ceff = feedback(C2, Gm0 - Gm);

% 1) Sensitivity peak Ms (lower = more robust, 1.2 to 2 is typical)
w  = logspace(-3, 1, 3000);
S1 = 1./(1 + squeeze(freqresp(C1*Gp*Gs,   w)));
S2 = 1./(1 + squeeze(freqresp(Ceff*Gp*Gs, w)));
fprintf('Sensitivity peak Ms:  PI = %.2f   PI+Smith = %.2f\n', ...
        max(abs(S1)), max(abs(S2)));

% 2) Valve activity caused by RTD measurement noise (0.05 degC rms)
t = (0:0.5:600)';
n = 0.05*randn(size(t));
u1 = lsim(-feedback(C1,   Gp*Gs), n, t);
u2 = lsim(-feedback(Ceff, Gp*Gs), n, t);
fprintf('Valve movement from noise (std, %% open):  PI = %.3f   PI+Smith = %.3f\n', ...
        std(u1), std(u2));

plot(t, u1, t, u2); grid on;
legend('PI', 'PI + Smith predictor');
title('Valve movement caused by sensor noise');
xlabel('Time (s)'); ylabel('Valve command change (%)');