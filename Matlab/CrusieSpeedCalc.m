%% Ideal Cruise Speed Sizing (Fire Suppression <= 3 min)


clear; clc; close all;

%% ----  INPUTS ----
T_total_s        = 180;     % total allowed time (s)  (3 min = 180 s)
D_miles          = 1.35;    % one-way distance to target (miles)

t_delivery_min_s = 40;      % payload delivery lower bound (s)
t_delivery_max_s = 50;      % payload delivery upper bound (s)

t_overhead_s     = 5;       % optional overhead time (s): takeoff + climb + align, etc.


%% ---- SINGLE-POINT: Worst-case delivery time ----
t_delivery_worst_s = t_delivery_max_s;

t_cruise_s = T_total_s - t_overhead_s - t_delivery_worst_s;

if t_cruise_s <= 0
    error('No time left for cruise. Reduce overhead/delivery time or increase total time budget.');
end

V_mph = D_miles * 1/t_cruise_s * 60^2;

fprintf('--- Ideal Cruise Speed Requirement (Worst-case delivery) ---\n');
fprintf('Total time limit:                  %.1f s\n', T_total_s);
fprintf('Distance to target:                %.3f miles\n', D_miles);
fprintf('Overhead time (like Take off):     %.1f s\n', t_overhead_s);
fprintf('Delivery time (worst):             %.1f s\n', t_delivery_worst_s);
fprintf('Time available for cruise:         %.1f s\n', t_cruise_s);
fprintf('Required cruise speed:             %.2f mph\n\n',V_mph);