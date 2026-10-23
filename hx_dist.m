clear; clc; close all;
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;

Gp = tf(1.5, [50 1], 'InputDelay', 15);

% Controller 1: plain PI
Kc1 = tau/(K*(2*theta));  Ti1 = min(tau, 8*theta);
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);

% Controller 2: PI + Smith predictor
Kc2 = tau/(K*10);  Ti2 = tau;
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);
Gm0 = tf(K, [tau 1]);
Gm  = tf(K, [tau 1], 'InputDelay', theta);
Ceff = feedback(C2, Gm0 - Gm);

% Response of outlet temperature to a 5 % input disturbance
t  = (0:0.5:600)';
y1 = step(5*feedback(Gp, C1),   t);
y2 = step(5*feedback(Gp, Ceff), t);

plot(t,y1,t,y2); grid on;
legend('PI','PI + Smith predictor');
title('Load disturbance rejection (5 % input step)');
xlabel('Time (s)'); ylabel('Temperature deviation (degC)');

fprintf('%-12s | peak dev | recovery (<0.5 degC) | IAE\n','Controller');
for k = 1:2
    y = {y1,y2}; nm = {'PI','PI+Smith'};
    idx = find(abs(y{k}) > 0.5, 1, 'last');
    if isempty(idx), tr = 0; else, tr = t(min(idx+1,numel(t))); end
    fprintf('%-12s | %6.2f   | %8.0f s          | %6.1f\n', ...
        nm{k}, max(abs(y{k})), tr, trapz(t,abs(y{k})));
end