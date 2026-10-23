clear; clc; close all;
load hx_est.mat est               % identified model

% Identified first-order-plus-dead-time parameters
K = est.Kp;  tau = est.Tp1;  theta = est.Td;

% True plant (used only for simulation)
Gp = tf(1.5, [50 1], 'InputDelay', 15);

% SIMC PI tuning rule, closed-loop time constant tc = theta
tc = theta;
Kc = tau / (K*(tc + theta));
Ti = min(tau, 4*(tc + theta));
C  = tf(Kc*[Ti 1], [Ti 0]);       % PI controller

fprintf('PI controller: Kc = %.3f   Ti = %.1f s\n', Kc, Ti);

% Closed loop and step response to a 10 degC setpoint change
T = feedback(C*Gp, 1);
step(10*T, 300); grid on;
title('Closed loop with PI controller: 10 degC setpoint step');
xlabel('Time'); ylabel('Outlet temperature rise (degC)');

S = stepinfo(T);
fprintf('Overshoot = %.1f %%   Settling time = %.0f s\n', ...
        S.Overshoot, S.SettlingTime);