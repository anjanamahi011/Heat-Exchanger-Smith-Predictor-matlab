clear; clc; close all;
load hx_est.mat est
K = est.Kp;  tau = est.Tp1;  theta = est.Td;

% Controller 1: plain PI (SIMC, tc = theta)
Kc1 = tau/(K*(2*theta));  Ti1 = min(tau, 8*theta);
C1  = tf(Kc1*[Ti1 1], [Ti1 0]);

% Controller 2: PI + Smith predictor (tc = 10 s)
Kc2 = tau/(K*10);  Ti2 = tau;
C2  = tf(Kc2*[Ti2 1], [Ti2 0]);
Gm0 = tf(K, [tau 1]);
Gm  = tf(K, [tau 1], 'InputDelay', theta);
Ceff = feedback(C2, Gm0 - Gm);

% Test cases: gain factor, delay factor
names = {'Nominal','Delay +30%','Delay -30%','Gain +30%','Gain -30%'};
kf = [1 1 1 1.3 0.7];
df = [1 1.3 0.7 1 1];

t = (0:0.5:600)';
fprintf('%-12s | %-22s | %-22s\n','Case','PI: OS% / Ts / IAE','PI+Smith: OS% / Ts / IAE');
for i = 1:5
    Gp = tf(1.5*kf(i), [50 1], 'InputDelay', 15*df(i));
    y1 = step(10*feedback(C1*Gp,1),   t);
    y2 = step(10*feedback(Ceff*Gp,1), t);
    r1 = metrics(y1,t);  r2 = metrics(y2,t);
    fprintf('%-12s | %5.1f / %5.0f / %6.1f | %5.1f / %5.0f / %6.1f\n', ...
        names{i}, r1(1),r1(2),r1(3), r2(1),r2(2),r2(3));
    subplot(2,3,i); plot(t,y1,t,y2); grid on; xlim([0 300]);
    title(names{i}); ylabel('degC');
    if i==1, legend('PI','PI+Smith','Location','southeast'); end
end

function r = metrics(y,t)
    os  = max(0, (max(y)-10)/10*100);
    idx = find(abs(y-10) > 0.5, 1, 'last');    % last time outside 5 % band
    if isempty(idx), ts = 0; else, ts = t(min(idx+1,numel(t))); end
    if abs(y(end)-10) > 0.5, ts = NaN; end      % did not settle
    r = [os, ts, trapz(t, abs(10-y))];
end