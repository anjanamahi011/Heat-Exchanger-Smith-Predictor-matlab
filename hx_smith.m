clear; clc; close all;
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;

% True plant
Gp = tf(1.5, [50 1], 'InputDelay', 15);

% ---- Controller 1: plain PI (SIMC, tc = theta) ----
tc1 = theta;
Kc1 = tau/(K*(tc1+theta));  Ti1 = min(tau, 4*(tc1+theta));
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);
T1  = feedback(C1*Gp, 1);

% ---- Controller 2: PI + Smith predictor (faster tuning) ----
tc2 = 10;                                % closed-loop time constant, s
Kc2 = tau/(K*tc2);  Ti2 = tau;           % cancels the plant pole
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);

Gm0 = tf(K, [tau 1]);                    % model WITHOUT delay
Gm  = tf(K, [tau 1], 'InputDelay', theta); % model WITH delay
Ceff = feedback(C2, Gm0 - Gm);           % Smith predictor structure
T2   = feedback(Ceff*Gp, 1);

% ---- Compare ----
step(10*T1, 10*T2, 300); grid on;
legend('PI (SIMC)', 'PI + Smith predictor');
title('Setpoint step: PI vs PI with Smith predictor');
xlabel('Time'); ylabel('Outlet temperature rise (degC)');

S1 = stepinfo(T1);  S2 = stepinfo(T2);
fprintf('PI           : Overshoot = %.1f %%   Settling = %.0f s\n', S1.Overshoot, S1.SettlingTime);
fprintf('PI + Smith   : Overshoot = %.1f %%   Settling = %.0f s\n', S2.Overshoot, S2.SettlingTime);