clear; clc; close all;
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;
Gs = tf(1, [5 1]);                           % RTD sensor lag, 5 s

% Controller 1: PI, SIMC with sensor lag added to the delay
thetaE = theta + 5;
Kc1 = tau/(K*(2*thetaE));  Ti1 = min(tau, 8*thetaE);
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);

% Controller 2: PI + Smith predictor, model includes the sensor lag
Kc2 = tau/(K*10);  Ti2 = tau;
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);
Gm0 = tf(K, [tau 1]) * Gs;                          % delay-free part
Gm  = tf(K, [tau 1], 'InputDelay', theta) * Gs;     % full model
Ceff = feedback(C2, Gm0 - Gm);

cases = {'Nominal','Delay +30%','Delay -30%','Gain +30%','Gain -30%'};
kf = [1 1 1 1.3 0.7];
df = [1 1.3 0.7 1 1];
Ctrl = {C1, Ceff};  nm = {'PI','PI+Smith'};
t = (0:0.5:800)';

rows = [];
for i = 1:5
    Gp = tf(1.5*kf(i), [50 1], 'InputDelay', 15*df(i));
    for k = 1:2
        yr = step(10*feedback(Ctrl{k}*Gp, Gs), t);
        yd = step(5*feedback(Gp, Ctrl{k}*Gs), t);
        [os, ts, iae] = sp_metrics(yr, t);
        rows = [rows; i k os ts iae max(abs(yd))]; %#ok<AGROW>
    end
end

Tbl = table(string(cases(rows(:,1)))', string(nm(rows(:,2)))', ...
    rows(:,3), rows(:,4), rows(:,5), rows(:,6), ...
    'VariableNames', {'Case','Controller','Overshoot_pct','Settling_s', ...
                      'IAE_setpoint','Dist_peak_degC'});
disp(Tbl);
writetable(Tbl, 'results2.csv');

Gp = tf(1.5, [50 1], 'InputDelay', 15);
subplot(1,2,1);
plot(t, step(10*feedback(C1*Gp, Gs), t), t, step(10*feedback(Ceff*Gp, Gs), t));
grid on; xlim([0 300]); legend('PI','PI + Smith');
title('Setpoint step (nominal)'); xlabel('Time (s)'); ylabel('degC');
subplot(1,2,2);
plot(t, step(5*feedback(Gp, C1*Gs), t), t, step(5*feedback(Gp, Ceff*Gs), t));
grid on; xlim([0 400]); legend('PI','PI + Smith');
title('Load disturbance (nominal)'); xlabel('Time (s)'); ylabel('degC');

function [os, ts, iae] = sp_metrics(y, t)
    os  = max(0, (max(y) - 10)/10*100);
    idx = find(abs(y - 10) > 0.5, 1, 'last');
    if isempty(idx), ts = 0; else, ts = t(min(idx+1, numel(t))); end
    if abs(y(end) - 10) > 0.5, ts = NaN; end
    iae = trapz(t, abs(10 - y));
end