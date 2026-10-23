clear; clc; close all;

% Assumed heat exchanger parameters (simulated plant)
K     = 1.5;   % process gain, degC per % valve
tau   = 50;    % time constant, s
theta = 15;    % transport delay, s

Gp = tf(K, [tau 1], 'InputDelay', theta);

% Open-loop response to a 10 % valve step
step(10*Gp, 300); grid on;
title('Open-loop response to a 10 % valve step');
xlabel('Time'); ylabel('Outlet temperature rise (degC)');