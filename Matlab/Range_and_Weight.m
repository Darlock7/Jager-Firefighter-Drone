%% Constants
g = 9.80665;
lb2N = 4.44822;

%% Given
W0_lb = 55;                 % gross weight guess [lb]
W0 = W0_lb * lb2N;          % [N]

Wp = ;                      % payload weight [N]
ws_wg = ;                   % structure weight fraction Ws/Wg

A = ;                       % empirical coefficient
C = ;                       % empirical exponent
Kvs = ;                     % configuration factor

hb = ;                      % motor/prop efficiency
eb = ;                      % battery efficiency
wb = ;                      % battery specific energy [J/kg]

L = ;                       % lift [N]
D = ;                       % drag [N]
V = ;                       % cruise speed [m/s]

%% Empty weight fraction 
we_w0 = A * (W0_lb^C) * Kvs;

%% Convert to We/Wg form
we_wg = we_w0;              % W0 ≈ Wg for first iteration

%% Gross weight
Wg = Wp / (1 - we_wg - ws_wg);

%% Battery weight
Wb = Wg - (we_wg * Wg) - (ws_wg * Wg) - Wp;

%% Range (electric aircraft)
R = (hb * eb * wb / g) * (L / D);

