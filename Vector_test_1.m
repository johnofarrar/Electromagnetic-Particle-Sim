%% CHANGELOG
% added support for multiple particles
% added particle trajectory graphing
    % fig1: dot added for each timestep
    % fig2: dot trajectory animated

%# initialize
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


%% PARTICLE SETUP (all x, y, z)

% NUMBER OF PARTICLES
particle_Count = 3;

% Mass array (kg)
mass = [1, 1];

% Charge array (C)
Q = [1 , -1, 0];

% color array based off charge
color = zeros(particle_Count,3);
for i = 1:particle_Count
    if Q(i) > 0
        color(i,:) = [1, 0, 0];
    elseif Q(i) < 0
        color(i,:) = [0, 0, 1];
    else
        color(i,:) = [.8, .4, .8];
    end
end


% Position matrix (meters)
pos_Vec = [50,0,0; 0,50,0; 0,0,50];
% velocity vector (meters/second)
vel_Vec = [-10,0,5; 0,0,0; -1,-1,-1];
% acceleration vector (meters/second^2)
accel_Vec = [2,2,0; 0,0,0; -1,-1,-1];


%% SIMULATE

% time vector
time = [0:delta_t:(delta_t * tick_Num_Total)];

for tick_Num = 0:tick_Num_Total
    % calculate current time
    time = (tick_Num * delta_t);
    
    % determine if display is true
    display_On_Tick = (mod(tick_Num, disp_Interval) == 0);
    
    % print time
    if (display_On_Tick)
        fprintf('================================\n--------------------------------\nTIME (seconds): %.3f\n', time)
    end

    % Loop over all particles
    for particle_Num = 1:particle_Count

        if (display_On_Tick)
            % Print vector values
            fprintf('--------------------------------\nPARTICLE %d\nPosition: [%.2f, %.2f, %.2f]\nVelocity: [%.1f, %.1f, %.1f]\n', particle_Num, pos_Vec(particle_Num, 1), pos_Vec(particle_Num, 2), pos_Vec(particle_Num, 3), vel_Vec(particle_Num, 1), vel_Vec(particle_Num, 2), vel_Vec(particle_Num, 3))
        end
    
        % calculate vector values for next tick
        
        pos_Vec(particle_Num, :) = (pos_Vec(particle_Num, :) + (vel_Vec(particle_Num, :) * delta_t));
        vel_Vec(particle_Num, :) = (vel_Vec(particle_Num, :) + (accel_Vec(particle_Num, :) * delta_t));

        
    end

    pos_all(:,:, tick_Num + 1) = pos_Vec;
    vel_all(:,:, tick_Num + 1) = vel_Vec;
end

%% PLOT

% Plot dots for each time step
figure(1) 
for t_Index = 1:50:(tick_Num_Total + 1)
    for N = 1:particle_Count
        
        % size
        S = (t_Index * (40/tick_Num_Total));
        
        scatter3(pos_all(N, 1,t_Index), pos_all(N, 2,t_Index), pos_all(N, 3,t_Index), S, color(N, :), 'filled')
        hold on
        
        grid on
        xlabel('x')
        ylabel('y')
        zlabel('z')
        title((t_Index * delta_t), 'SECONDS')
        view([45, 45])

    end
end


% Movie
figure(2) 
% size
S = 5;

% ANIMATION TIMESTEP
animation_Timestep = 10;

% start frame index
frameIndex = 1;

% preallocate for movieVector
movieVector(frameIndex) = struct('cdata', [], 'colormap', []);

for t_Index = 1:animation_Timestep:(tick_Num_Total + 1)
    
    % clear plot
    clf;

    % graph settings
        hold on
        grid on
        xlabel('x')
        ylabel('y')
        zlabel('z')
        title((t_Index * delta_t), 'SECONDS')
        view([45, 45])

    for N = 1:particle_Count
        % plot data
        plot3(pos_all(N,1,t_Index), pos_all(N,2,t_Index), pos_all(N,3,t_Index), 'o', 'MarkerSize', S, 'MarkerFaceColor', color(N,:), 'MarkerEdgeColor', color(N,:))
    end
    drawnow
    movieVector(frameIndex) = getframe(gcf);  % assign frame using consecutive index
    frameIndex = frameIndex + 1;  
end

%% set up video_Writer
video_Writer = VideoWriter('ElectroMag_Particle_Sim');
video_Writer.FrameRate = 20;

% write video
open(video_Writer);
writeVideo(video_Writer, movieVector);
close(video_Writer);