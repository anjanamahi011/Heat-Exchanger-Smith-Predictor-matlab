clear; clc; close all;
rng(1);                                % repeatable noise

% True plant (what the "real" exchanger does)
K = 1.5; tau = 50; theta = 15;
Gp = tf(K, [tau 1], 'InputDelay', theta);

% Step test: valve moves 0 -> 10 % at t = 10 s
Ts = 1;
t  = (0:Ts:400)';
u  = 10*(t >= 10);
y  = lsim(Gp, u, t);
y  = y + 0.05*randn(size(y));          % RTD measurement noise (degC)

% Fit first-order-plus-dead-time model: K/(tau*s+1)*exp(-theta*s)
data = iddata(y, u, Ts);
est  = procest(data, 'P1D');

fprintf('Identified:  K = %.3f   tau = %.1f s   delay = %.1f s\n', ...
        est.Kp, est.Tp1, est.Td);
fprintf('True:        K = %.3f   tau = %.1f s   delay = %.1f s\n', K, tau, theta);

compare(data, est); grid on;
title('Step-test data vs identified model');

% Save the identified model; the other hx_*.m scripts load it
save('hx_est.mat', 'est');
