%% setup_guardian_mission.m
%
% Combined base-workspace setup for the Guardian_Mission harness --
% prepares BOTH Guardian_Battery's and Guardian_Top's data and bus
% definitions in one script, run once before opening/simulating
% Guardian_Mission.slx.
%
% DELIBERATELY DOES NOT CONTROL THE MODEL. This is data preparation only
% -- no sim(), no set_param() on model blocks, no simulation loop. The
% original Guardian_Top_session_for_publish_script.m's gain-calibration
% loop (set_param + sim + plot, sweeping right_gain_calibration) has been
% REMOVED on purpose, per instruction: Guardian_Mission runs purely from
% Simulink, driven by ordinary block-diagram wiring and Configuration
% Parameters, never by a script stepping the model in a loop.

clearvars
close all
clc

%% Guardian Mission file
Ts = 0.1;
tau_current = 2;   % seconds -- how long current takes to settle to a new commanded level (estimate, tune freely)
a_current   = exp(-Ts/tau_current);
Numerator_speed_current = 1-a_current;
Denominator_speed_current = [1, -a_current];
SOC_Critical = double(5);
SOC_Warn = double(30);
SOH_Min = double(90);
V_min = double(2.5);
vx_FORWARD = double(0.5);
vx_REDUCED = double(0.2);
omega_TURN = double(0.5);
Charging_Current = -25;
a_turn = exp(-0.1/0.5);

%% ============================================================
%% GUARDIAN_BATTERY DATA (unchanged from Guardian_Battery_script.m)
%% ============================================================

driveCycleTbl = readtable('Guardian_Battery_DriveCycle_Current (1)');
ocvLookupTbl  = readtable('Guardian_Battery_OCV_SOC_Table (1)');

t_vec_s   = driveCycleTbl.Time_s;
I_drive_A = driveCycleTbl.Current_A;

SOC_breakpoints = ocvLookupTbl.SOC_pct;
OCV_data = ocvLookupTbl.OCV_V;

I_workspace = [t_vec_s , I_drive_A];

% This becomes the Guardian_Mission harness's own Stop Time -- per
% instruction, the combined run is expanded to match the battery's full
% drive-cycle length, not Guardian_Top's original 100s test window.
T_end_s = max(t_vec_s);

%% Battery ECM constants (unchanged)

Q_nominal = 50; % Ah
R0_ref = 0.0007; % ohm
R1_ref = 0.0016; % ohm
C1_ref = 17000; % F
R2_ref = 0.00035; % ohm
C2_ref = 5200; % F
tau1_s = R1_ref * C1_ref;
tau2_s = R2_ref * C2_ref;
a1_coef = exp(-Ts/(tau1_s));
a2_coef = exp(-Ts/(tau2_s));
SOC_init_pct = 80;

%% Hysteresis Noise Filter (FR-3, rescaled for Ts) -- unchanged

Ts_hyst_ref     = 1;
a_hyst_baseline = 0.98;
a_hyst_coef     = a_hyst_baseline ^ (Ts / Ts_hyst_ref);
hyst_filt_num   = 1 - a_hyst_coef;
hyst_filt_den   = [1, -a_hyst_coef];

%% Guardian_Battery bus objects (unchanged)

busElems(1) = Simulink.BusElement;
busElems(1).Name = 'SOC_CC'; busElems(1).DataType = 'double';
busElems(2) = Simulink.BusElement;
busElems(2).Name = 'SOC_Flag'; busElems(2).DataType = 'boolean';
busElems(3) = Simulink.BusElement;
busElems(3).Name = 'SOC_hat'; busElems(3).DataType = 'double';
busElems(4) = Simulink.BusElement;
busElems(4).Name = 'V1_hat'; busElems(4).DataType = 'double';
busElems(5) = Simulink.BusElement;
busElems(5).Name = 'V2_hat'; busElems(5).DataType = 'double';
busElems(6) = Simulink.BusElement;
busElems(6).Name = 'R0_TLS'; busElems(6).DataType = 'double';
busElems(7) = Simulink.BusElement;
busElems(7).Name = 'SOH_ratio'; busElems(7).DataType = 'double';
busElems(8) = Simulink.BusElement;
busElems(8).Name = 'V_terminal'; busElems(8).DataType = 'double';

GuardianCellBus = Simulink.Bus;
GuardianCellBus.Elements = busElems;
% Inner Cell_data bus -- Cell_DigitalTwin's own internal wiring only.

busElemsPublic(1) = Simulink.BusElement;
busElemsPublic(1).Name = 'R0_TLS'; busElemsPublic(1).DataType = 'double';
busElemsPublic(2) = Simulink.BusElement;
busElemsPublic(2).Name = 'SOH_ratio'; busElemsPublic(2).DataType = 'double';
busElemsPublic(3) = Simulink.BusElement;
busElemsPublic(3).Name = 'SOC_EKF'; busElemsPublic(3).DataType = 'double';
busElemsPublic(4) = Simulink.BusElement;
busElemsPublic(4).Name = 'SOC_Flag'; busElemsPublic(4).DataType = 'boolean';
busElemsPublic(5) = Simulink.BusElement;
busElemsPublic(5).Name = 'V_terminal'; busElemsPublic(5).DataType = 'double';

BatteryDataBus = Simulink.Bus;
BatteryDataBus.Elements = busElemsPublic;
% Public bus -- what's actually wired into Guardian_Battery's own
% Battery_data outport. This is what Guardian_Mission's Mission_Supervisor
% will pull from via a Bus Selector.

%% ============================================================
%% GUARDIAN_TOP DATA (bus definition only -- calibration loop removed)
%% ============================================================
% Example sensor test signals -- FIXED: the original version only
% covered t=5 to t=100s, but the harness now runs for the full T_end_s
% (~3600s, matching the battery drive cycle). Left as-is, From Workspace
% would either extrapolate the ramp indefinitely (unrealistic) or hold
% the last value flat (harmless but inert) for the remaining ~97% of the
% run -- either way, Debounce_healer would go essentially untested for
% almost the whole harness run. Fixed by repeating the exact same
% obstacle-encounter shape across the full duration instead of inventing
% a new signal design. Replace with your own scenario if this doesn't
% match what you want Guardian_Mission's sensors to see -- nothing below
% this comment drives or steps the model either way.

cycle_v   = [5*ones(300,1); linspace(5,2,300)'; 2*ones(351,1)];  % one encounter, unchanged shape
cycle_len = numel(cycle_v);
n_reps    = ceil(T_end_s / (cycle_len * Ts)) + 1;                % Ts=0.1 from the battery section above

v_sensor_full = repmat(cycle_v, n_reps, 1);
t_sensor_full = (0:numel(v_sensor_full)-1)' * Ts;

keep = t_sensor_full <= T_end_s;                                 % trim to exactly T_end_s
t_sensor = t_sensor_full(keep);
v_sensor = v_sensor_full(keep);

right_gain = [t_sensor , v_sensor];
left_gain  = [t_sensor , v_sensor];
front_gain = [t_sensor , v_sensor];

% Guardian_Top's own bus object -- unchanged from
% Guardian_Top_session_for_publish_script.m
elems(1) = Simulink.BusElement;
elems(1).Name = 'Distance_cm'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement;
elems(2).Name = 'Close_Flag'; elems(2).DataType = 'boolean';
elems(3) = Simulink.BusElement;
elems(3).Name = 'Fault_Flag'; elems(3).DataType = 'boolean';
SensorDataBus = Simulink.Bus;
SensorDataBus.Elements = elems;
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'STOP_Path'; elems(4).DataType = 'double';
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'Stop_String'; elems(5).DataType = 'string';
% SensorDataBus = Simulink.Bus;
% SensorDataBus.Elements = elems;

% debouncehealer parameters
N_confirm = 3;
N_heal = 3;

%% ============================================================
%% Report
%% ============================================================
fprintf('\n--- Guardian_Mission base-workspace setup complete ---\n');
fprintf('Battery drive cycle: %d samples, %.1fs (this is your harness Stop Time, T_end_s)\n', ...
    numel(t_vec_s), T_end_s);
fprintf('Battery Ts = %.2fs (fixed-step discrete)\n', Ts);
fprintf('Bus objects in base workspace: GuardianCellBus, BatteryDataBus, SensorDataBus\n');
fprintf('Sensor test signals available: right_gain, left_gain, front_gain\n');
fprintf('  -- %d obstacle-encounter cycles repeated across the full %.0fs run (not just the first 100s)\n', floor(numel(t_sensor)/cycle_len), T_end_s);
fprintf('\nNothing in this script calls sim() or set_param() on a model --\n');
fprintf('run and observe Guardian_Mission.slx directly in Simulink.\n');

%% scenario_FINAL_ceremony.m
%
% CORRECTED from the previous version: no separate/bypassing current
% signal. Speed_cmd -> LoadModel -> current_selector_module stays
% completely real and untouched throughout. Confirmed real Speed_cmd
% values per state: IDLE=50, FORWARD=100, TURN(L or R)=25, STOP=0 --
% run through LoadModel's own real table to get real currents:
%   IDLE=20A, FORWARD=40A, TURN=10A, STOP=0A
%
% Demo pacing is achieved the honest way: Q_nominal temporarily reduced
% to 12 Ah for this run only (real battery capacity parameter, nothing
% to do with the current-generation pipeline). Verified in Python:
% reproduces almost exactly the same milestone timeline as the earlier,
% incorrect 150A-bypass version (LOW_POWER~630s, CELL_FAULT~846s,
% RTB_STOP~900s) but now through the fully real pipeline.
%
% SET THIS IN setup_guardian_mission.m BEFORE RUNNING:
%   Q_nominal = 12;   % temporary, THIS DEMO ONLY -- revert to 50 after

Ts = 0.1;
T_total = 1000;
t = (0:Ts:T_total)';

V_bp = [0 1 2 3 4 5];
D_table = [400 300 200 100 50 10];

% ---- Sensor voltages (drives Active_State_dis -> Speed_cmd -> real
% current, AND the visualization, all from the same real signal) ----
% [t_start, t_end, Front_Dist, Right_Dist, Left_Dist]
motion_segs = {
      0, 20, 80, 100, 100;   % FORWARD
      20, 23, 25, 80, 20;   % TURN_R
      23, 60, 80, 100, 100;   % FORWARD
      60, 63, 25, 20, 80;   % TURN_L
      63, 900, 80, 100, 100;   % FORWARD (long cruise -- dominant depletion driver)
};
% t=900-950s: RTB_STOP forces the car to stop regardless of sensors;
% sensors held at FORWARD through this gap, which is exactly the point
% being demonstrated (supervisor overrides sensor commands).

Vf = dist_to_voltage(80, V_bp, D_table) * ones(size(t));
Vr = dist_to_voltage(100, V_bp, D_table) * ones(size(t));
Vl = dist_to_voltage(100, V_bp, D_table) * ones(size(t));
for i = 1:size(motion_segs,1)
    idx = t >= motion_segs{i,1} & t < motion_segs{i,2};
    Vf(idx) = dist_to_voltage(motion_segs{i,3}, V_bp, D_table);
    Vr(idx) = dist_to_voltage(motion_segs{i,4}, V_bp, D_table);
    Vl(idx) = dist_to_voltage(motion_segs{i,5}, V_bp, D_table);
end

% ---- Buttons ----
Estop = zeros(size(t));
Estop(t >= 250 & t < 250+Ts) = 1;   % manual stop while healthy (SOC~74% at t=250s)

Reset = zeros(size(t));
Reset(t >= 260 & t < 260+Ts) = 1;    % quick resume after manual stop
Reset(t >= 901 & t < 901+Ts) = 1;    % PREMATURE -- must bounce back
Reset(t >= 950 & t < 950+Ts) = 1;    % successful -- recovered (~7.9%, faster now at Q=12)

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(timeseries(Vf, t),    'Voltage_enter_front');
ds = ds.addElement(timeseries(Vr, t),    'Voltage_enter_right');
ds = ds.addElement(timeseries(Vl, t),    'Voltage_enter_left');
ds = ds.addElement(timeseries(Estop, t), 'Emergency_Stop_Cmd');
ds = ds.addElement(timeseries(Reset, t), 'Reset_Cmd');

save('scenario_FINAL_ceremony.mat', 'ds');

%% Report
fprintf('\n=== FINAL CEREMONY -- %ds, real pipeline throughout ===\n\n', T_total);
fprintf('BEFORE RUNNING: set Q_nominal=12 in setup_guardian_mission.m (temporary,\n');
fprintf('this run only -- revert to 50 afterward).\n\n');
fprintf('t=  0- 180s : FORWARD -> TURN_R -> TURN_L (watch the car respond)\n');
fprintf('t=180- 250s : FORWARD continues, real current draining SOC quietly\n');
fprintf('t=     250s : Emergency_Stop_Cmd -- instant stop, SOC still healthy (~74%%)\n');
fprintf('t=     260s : Reset_Cmd -- clean resume\n');
fprintf('t=260- 900s : FORWARD continues\n');
fprintf('t=    ~630s : expect LOW_POWER\n');
fprintf('t=    ~846s : expect CELL_FAULT\n');
fprintf('t=    ~900s : expect RTB_STOP -- car stops, charging auto-engages (-25A, no button)\n');
fprintf('t=     901s : Reset_Cmd PREMATURE -- *** watch this one -- must bounce back ***\n');
fprintf('t=     950s : Reset_Cmd -- SOC recovered ~7.9%%, expect successful resume\n');

function v = dist_to_voltage(d, V_bp, D_table)
    d = max(10, min(400, d));
    v = interp1(fliplr(D_table), fliplr(V_bp), d, 'linear');
end