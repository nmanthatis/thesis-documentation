clear; clc;

%========================= Motor - Propeller Analysis ==========================
% This script has two functionalities. 
% 
% 1.
% The 1st functionality is based on monitoring the efficiency values for the 
% propeller and the motor and the output thrust of the propeller for steady
% state condition (Moment_propeller == Moment_motor). For the 1st you have to 
% define the specific flight velocity (Vin) and voltage applied to the motor (vin) 
% 
% 2.
% The 2nd one is for identifying the necessary motor voltage for a desired
% Thrust output and a specified flight velocity again for steady state condition
%
% CONDITIONS FOR USING:
% Have a approximate function for Ct and Cn in relation to the advance ratio. 
% Ex: Ct = a*J + b
%===============================================================================

%================================= User Input ==================================

% if you want to print initial Motor and Propeller plots
pp = false;
pm = false;

% Define flight velocity and voltage for 1st functionality
V1in = 2;           % velocity 
v1in = 11;           % voltage  

% Define Thrust and flight velocity for 2nd functionality
Tin  = 1.25;           % desired Thrust
V2in = 16.67;           % velocity  

%------------------------------ Motor Parameters -------------------------------
i0 = 1.5;           % no load current [A]
R = 39.1e-3;        % internal resistance [Ohm]
kv = 1200;          % constant RPM/Volt

% Volt and angular vel
v = 1:0.01:12;         % specify different volt values [Volt]
w = 50:0.1:1500;  % specify different ang. vel. [rpm]

%---------------------------- Propeller Parameters -----------------------------
rho = 1.225;        % air density [kg/m^3]
D = 9 * 0.0254;     % propeller diameter [m]

%------ Flight Velocity [m/s]
V = 0:0.02:20;       % specify different velocity values

%------ Advance Ratio J and thrust, torque coefficients Ct, Cn
% the below constants is for approximate function Ct = ct1 * J^pt + ct2
% the same for Cn coefficient
ct1 = -0.050337553133783;
ct2 = 0.002689386692735;
pt  = 1.3;
cn1 = -0.061939927919731;
cn2 = 0.000157567049364;
pn  = 2.7;
%===============================================================================

%================================ Basic Plots ==================================
% w = w.*pi/30;
kv = kv*pi/30;
kt = 1/kv;

% Moment, Power and Efficiency of Motor
N = zeros(length(v), length(w));
Pm = zeros(length(v), length(w));
nm = zeros(length(v), length(w));

% Calculation for Motor
for i = 1:length(v)
    N(i, :) = kt * ((v(i) - w./kv).*1/R - i0);
    Pm(i, :) = ((v(i) - w./kv).*1/R - i0).*(w./kv);
    nm(i, :) = (1 - i0*R./(v(i) - w./kv)).*(w./(kv*v(i)));
end
N(N < 0) = 0;
Pm(Pm < 0) = 0;
nm(nm < 0) = 0;
nm(nm > 1) = 0;

% Plots for motor
if pm
    figure(1)
    hold on
    for i = 1:length(v)
        txt = ['v = ', num2str(v(i))];
        plot(w, N(i, :), 'DisplayName', txt)
    end
    hold off
    legend show
    xlabel('\omega [rad/s]')
    ylabel('N [Nm]')
    grid on
    title('Motor Torque vs \omega')
    

    figure(2)
    hold on
    for i = 1:length(v)
        txt = ['v = ', num2str(v(i))]; 
        plot(w, Pm(i, :), 'DisplayName', txt)
    end
    hold off
    legend show
    xlabel('\omega [rad/s]')
    ylabel('P_{m} [Watt]')
    grid on
    title('Mechanical Power of Motor vs \omega')

    figure(3)
    hold on
    for i = 1:length(v)
        txt = ['v = ', num2str(v(i))];
        plot(w, nm(i, :), 'DisplayName', txt)
    end
    legend show
    xlabel('\omega [rad/s]')
    ylabel('\eta_{m}')
    grid on
    title('Motor Efficiency vs \omega')
end

% Calculate Thrust and Moments for Propeller
J = zeros(length(V), length(w));
Ct = zeros(length(V), length(w));
Cn = zeros(length(V), length(w));
T = zeros(length(V), length(w));
M = zeros(length(V), length(w));
np = zeros(length(V), length(w));
for i = 1:length(V)
    J(i, :) = V(i)./(w.*D);
    Ct(i, :) = ct1.*J(i, :).^pt + ct2;
    Cn(i, :) = cn1.*J(i, :).^pn + cn2;
    T(i, :) = rho * D^4 .* Ct(i, :) .* w.^2;
    M(i, :) = rho * D^5 .* Cn(i, :) .* w.^2;
    np(i, :) = Ct(i, :) .* J(i, :) ./ (Cn(i, :));
end
M(M < 0) = 0;
T(T < 0) = 0; 
np(np < 0) = 0;
np(np > 1) = 0;

% Plots for Propeller
if pp
    figure(4)
    hold on 
    for i = 1:length(V)
        txt = ['V = ', num2str(V(i)), 'm/s'];
        plot(w, T(i, :), 'DisplayName', txt)
    end
    hold off
    legend show
    xlabel('\omega [rad/s]')
    ylabel('T [N]')
    title('Propeller Thrust vs \omega')

    figure(5)
    hold on 
    for i = 1:length(V)
        txt = ['V = ', num2str(V(i)), 'm/s'];
        plot(w, M(i, :), 'DisplayName', txt)
    end
    hold off
    legend show
    xlabel('\omega [rad/s]')
    ylabel('M [Nm]')
    title('Propeller Torque vs \omega')

    figure(6)
    hold on 
    for i = 1:length(V)
        txt = ['V = ', num2str(V(i)), 'm/s'];
        plot(w, np(i, :), 'DisplayName', txt)
    end
    hold off
    legend show
    xlabel('\omega [rad/s]')
    ylabel('\eta_{p}')
    title('Propeller Efficiency vs \omega')
end
%===============================================================================


%============================== 1st Functionality ==============================
%--------------------------- Intersections of Torques --------------------------

% index for motor
[d, k] = min(abs(v - v1in));
% index for propeller
[d, j] = min(abs(V - V1in));

[px, py] = intersections(w, M(j, :), w, N(k, :));
[d, i] = min(abs(w - px));

NameArray = {'XTick', 'XTickLabel'};
ValueArray = {100:100:1600, []};
figure(7)
h1 = subplot(4, 1, 1);
plot(w, T(j, :), px, T(j, i), 'ro')
ylabel('Thrust [N]')
grid on
set(gca, NameArray, ValueArray)

h2 = subplot(4, 1, 2);
plot(w, M(j, :), w, N(k, :), px, py, 'ro')
legend("Prop Torque at " + V(j) + "m/s", "Motor Torque at " + v(k) + "V")
ylabel('Torque [Nm]')
ylim([0 0.5])
str = {"\omega is " + w(i) + "rad/s", "Thrust is " + T(j, i) + "N",...
       "Motor efficiency is " + nm(k, i), "Propeller Efficiency is " + np(j, i)};
annotation('textbox', [0.75, 0.5, 0.2, 0.2], 'String', str, 'FitBoxToText','on')
grid on
set(gca, NameArray, ValueArray)

h3 = subplot(4, 1, 3);
plot(w, np(j, :), w(i), np(j, i), 'ro')
ylabel('\eta_{p}')
grid on
set(gca, NameArray, ValueArray)

h4 = subplot(4, 1, 4);
plot(w, nm(k, :), w(i), nm(k, i), 'ro')
xlabel('\omega [rad/s]')
ylabel('\eta_{m}')
grid on
set(gca, NameArray{1}, ValueArray{1})

set(h1, 'Position', [0.05, 0.77, 0.92, 0.22])
set(h2, 'Position', [0.05, 0.53, 0.92, 0.22])
set(h3, 'Position', [0.05, 0.29, 0.92, 0.22])
set(h4, 'Position', [0.05, 0.05, 0.92, 0.22])
% suptitle("Intersection Point for V = " + V(j) + "m/s and v = " + v(k) + "Volt")
%===============================================================================

%============================== 2nd Functionality ==============================
% index for propeller
[d, j] = min(abs(V - V2in));
% index for ang. vel.
[d, i] = min(abs(T(j, :) - Tin));
% propeller torque
Q = M(j, i);

[d, k] = min(abs(N(:, i) - M(j, i)));


figure(8);
%Styling
clr_prm1 = 'b'; clr_prm2 = 'r';
wdth_prm = 1.4;
clr_scd = 'k'; wdth_scd = 0.3;

h1 = subplot(4, 1, 1);
plot(w, T(j, :), clr_prm1, 'LineWidth', wdth_prm); hold on;
plot(w(i), T(j, i), 'ro');
p1 = [0 Tin]; p2 = [w(i) Tin]; dp = -p1 + p2;
line([p1(1) p2(1)], [p1(2) p2(2)], 'Color', clr_scd, 'LineWidth', wdth_scd);
q1 = quiver(p1(1), p1(2), dp(1)/2, dp(2)/2, 0);
p3 = [w(i), 0];
q1.MaxHeadSize = 0.025;
q1.Color = clr_scd; q1.LineWidth = wdth_scd;
ylabel('Thrust [N]')
xlim([200 inf])
grid on
set(gca, NameArray, ValueArray);

h2 = subplot(4, 1, 2);
plot(w, M(j, :), clr_prm1, 'LineWidth', wdth_prm); hold on;
plot(w, N(k, :), clr_prm2, 'LineWidth', wdth_prm);
plot(w(i), Q, 'ro')
legend("Prop Torque at " + V(j) + "m/s", "Motor Torque at " + v(k) + "V")
ylabel('Torque [Nm]')
ylim([0 0.5])
xlim([200 inf])
str = {"\omega is " + w(i) + "rad/s", "Torque is " + M(j, i) + "Nm",...
       "Motor efficiency is " + nm(k, i), "Propeller Efficiency is " + np(j, i),...
       "Voltage is " + v(k) + "V"};
annotation('textbox', [0.75, 0.5, 0.2, 0.2], 'String', str, 'FitBoxToText','on')
grid on
set(gca, NameArray, ValueArray)

h3 = subplot(4, 1, 3);
plot(w, np(j, :), clr_prm1, 'LineWidth', wdth_prm); hold on;
plot(w(i), np(j, i), 'ro');

p4 = [w(i) np(j, i)]; p5 = [0 np(j, i)]; dp = -p4 + p5;
line([p4(1) p5(1)], [p4(2) p5(2)], 'Color', clr_scd, 'LineWidth', 3.2*wdth_scd);
ylabel('\eta_{p}')
grid on
xlim([200 inf])
set(gca, NameArray, ValueArray)

[d, i200] = min(abs(w - 200));
h4 = subplot(4, 1, 4);
plot(w(i200:end), nm(k, i200:end),clr_prm1, 'LineWidth', wdth_prm); hold on;
plot(w(i), nm(k, i), 'ro');
p4 = [w(i) nm(k, i)]; p5 = [w(i200) nm(k, i)]; dp = -p4 + p5;
line([p4(1) p5(1)], [p4(2) p5(2)], 'Color', clr_scd, 'LineWidth', 3.2*wdth_scd);
xlabel('\omega [rad/s]')
ylabel('\eta_{m}')
grid on
xlim([200, inf])
set(gca, NameArray{1}, ValueArray{1})


set(gca,'Clipping','Off')
set(h1, 'Position', [0.05, 0.77, 0.92, 0.22])
set(h2, 'Position', [0.05, 0.53, 0.92, 0.22])
set(h3, 'Position', [0.05, 0.29, 0.92, 0.22])
set(h4, 'Position', [0.05, 0.05, 0.92, 0.22])
h = line([p2(1) p3(1)], [3.48 p3(2)]);
ylim([0 1]); set(h,'Color', 'k', 'LineStyle', '--', 'LineWidth', 0.6);
% suptitle("Intersection Point for V = " + V(j) + "m/s and Thrust = " + Tin + "N")
%==============================================================================%


% %for thesis plots
% [d, i1] = min(abs(v - 4));
% [d, i2] = min(abs(v - 7));
% [d, i3] = min(abs(v - 10));
% figure(10)
% hold on
% plot(w, N(i1, :), 'LineWidth', 2)str = {"\omega is " + w(i) + "rad/s", "Torque is " + M(j, i) + "Nm",...
%        "Motor efficiency is " + nm(k, i), "Propeller Efficiency is " + np(j, i),...
%        "Voltage is " + v(k) + "V"};
% annotation('textbox', [0.75, 0.5, 0.2, 0.2], 'String', str, 'FitBoxToText','on')
% plot(w, N(i3, :), 'k', 'LineWidth', 2)
% text(w(300)+20, N(i3, 300), 'v = 10', 'FontSize', 16)
% xlim([0 1300])
% hold off
% xlabel('\omega [rad/s]', 'FontSize', 16)
% ylabel('N [Nm]', 'FontSize', 16)
% grid on
% set(gca,'linewidth',1)
% title('Motor Torque vs \omega', 'FontSize', 20)

% figure(11)
% hold on
% plot(w, Pm(i1, :), 'LineWidth', 2)
% text(w(2000)+20, Pm(i1, 2000)+30, 'v = 4', 'FontSize', 16)
% plot(w, Pm(i2, :), 'LineWidth', 2)
% text(w(3000)+20, Pm(i2, 3000)+30, 'v = 7', 'FontSize', 16)
% plot(w, Pm(i3, :), 'k', 'LineWidth', 2)
% text(w(5000)+20, Pm(i3, 5000)+30, 'v = 10', 'FontSize', 16)
% xlim([0 1300])
% hold off
% xlabel('\omega [rad/s]', 'FontSize', 16)
% ylabel('P_{m} [Watt]', 'FontSize', 16)
% grid on
% set(gca,'linewidth',1)
% title('Mechanical Power of Motor vs \omega', 'FontSize', 20)

% figure(12)
% hold on
% plot(w, nm(i1, :), 'LineWidth', 2)
% text(w(3100)+40, nm(i1, 3000), 'v = 4', 'FontSize', 16)
% plot(w, nm(i2, :), 'LineWidth', 2)
% text(w(7000)+20, nm(i2, 7000), 'v = 7', 'FontSize', 16)
% plot(w, nm(i3, :), 'k', 'LineWidth', 2)
% text(w(10100)+30, nm(i3, 10000), 'v = 10', 'FontSize', 16)
% xlim([0 1300])
% hold off
% xlabel('\omega [rad/s]', 'FontSize', 16)
% ylabel('\eta_{m}', 'FontSize', 16)
% grid on
% set(gca,'linewidth',1)
% title('Motor Efficiency vs \omega', 'FontSize', 20)



[d, i1] = min(abs(V - 3));
[d, i2] = min(abs(V - 7));
[d, i3] = min(abs(V - 10));
figure(10)
hold on
plot(w, M(i1, :), 'LineWidth', 2)
text(w(3000)+20, M(i1, 3000)+0.025, 'V = 3', 'FontSize', 16)
plot(w, M(i2, :), 'LineWidth', 2)
annotation('textarrow', [0.35, 0.38], [0.26, 0.22], 'String', 'V = 7', 'FontSize', 16)
plot(w, M(i3, :), 'k', 'LineWidth', 2)
text(w(6000)+30, M(i3, 6000), 'V = 10', 'FontSize', 16)
xlim([200 inf])
hold off
xlabel('\omega [rad/s]', 'FontSize', 16)
ylabel('N [Nm]', 'FontSize', 16)
grid on
set(gca,'linewidth',1)
title('Propeller Torque vs \omega', 'FontSize', 20)
plot(w, T(i1, :), 'LineWidth', 2)
text(w(4000)+20, T(i1, 4000)+1.2, 'V = 3', 'FontSize', 16)
plot(w, T(i2, :), 'LineWidth', 2)
text(w(5000)+20, T(i2, 5000)+0.5, 'V = 7', 'FontSize', 16)
plot(w, T(i3, :), 'k', 'LineWidth', 2)
text(w(7000)+30, T(i3, 7000), 'V = 10', 'FontSize', 16)
xlim([200 inf])
hold off
xlabel('\omega [rad/s]', 'FontSize', 16)
ylabel('T [N]', 'FontSize', 16)
grid on
set(gca,'linewidth',1)
title('Propeller Thrust vs \omega', 'FontSize', 20)

figure(12)
hold on
plot(w, np(i1, :), 'LineWidth', 2)
text(w(5000)+10, np(i1, 5000)+0.02, 'V = 3', 'FontSize', 16)
plot(w, np(i2, :), 'LineWidth', 2)
text(w(7000)+40, np(i2, 7000), 'V = 7', 'FontSize', 16)
plot(w, np(i3, :), 'k', 'LineWidth', 2)
text(w(10100)+20, np(i3, 10100)+0.05, 'V = 10', 'FontSize', 16)
xlim([120 inf])
hold off
xlabel('\omega [rad/s]', 'FontSize', 16)
ylabel('\eta_{m}', 'FontSize', 16)
grid on
set(gca,'linewidth',1)
title('Propeller Efficiency vs \omega', 'FontSize', 20)