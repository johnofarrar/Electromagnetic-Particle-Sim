% initialize
clear
clc

%% TIME STEPS

% TOTAL SIM TIME (seconds)
time_Total = 10;
fprintf('Total Sim Time: %d\n', time_Total)

% TIMESTEP (seconds)
delta_t = .01;
fprintf('Tick Speed: %.3f\n', delta_t)

% DISPLAY TIME (seconds)
disp_t = 2;
fprintf('Display time: %.3f\n', disp_t)

% TOTAL TICKS (tics)
tick_Num_Total = fix(time_Total / delta_t);
fprintf('Total Ticks: %d\n', tick_Num_Total)

% Total Displays (n) and display interval (ticks / display)
disp_Total = (time_Total / disp_t);
disp_Interval = fix(tick_Num_Total / disp_Total);

% Initialize tick_Num = 0
tick_Num = 0;

%% PARTICLE VECTORS SETUP (all x, y, z)

% Position vector (meters)
pos_Vec = [0,0,0];

% velocity vector (meters/second)
vel_Vec = [1,2,3];

% acceleration vector (meters/second^2)
accel_Vec = [-1,-1,-1];

% Print initial vectors
fprintf('--------------------------------\nTime: 0\nPosition: [%.3f, %.3f, %.3f]\nVelocity: [%.3f, %.3f, %.3f]\n', pos_Vec(1), pos_Vec(2), pos_Vec(3), vel_Vec(1), vel_Vec(2), vel_Vec(3))

%% SIMULATE

for tick_Num = 1:tick_Num_Total
    % calculate current time
    time = (tick_Num * delta_t);
    
    if (mod(tick_Num, disp_Interval) == 0)
        % Print vector values
        fprintf('--------------------------------\nTime: %.3f\nPosition: [%.3f, %.3f, %.3f]\nVelocity: [%.3f, %.3f, %.3f]\n', time, pos_Vec(1), pos_Vec(2), pos_Vec(3), vel_Vec(1), vel_Vec(2), vel_Vec(3))
    end

    % calculate vector values for next tick
    pos_Vec = (pos_Vec + (vel_Vec * delta_t));
    vel_Vec = (vel_Vec + (accel_Vec * delta_t));

    % increment tick_Num
    tick_Num = (tick_Num + 1);
end