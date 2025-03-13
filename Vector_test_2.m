%% CHANGELOG
% added particle simulation movie
% removed graphing feature

%% initialize
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
fprintf('Total Ticks: %d\n\n', tick_Num_Total)

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

% NOTE: add a "render settings" display to the terminal

% Movie
figure
% size
S = 5;

% ANIMATION TIMESTEP
animation_Timestep = 5;

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
        axis([-100 100 -100 100 -100 100]) % Note to self: add a feture to dynamically change axis limits based on max value in pos arrays

    for N = 1:particle_Count
        % plot data
        plot3(pos_all(N,1,t_Index), pos_all(N,2,t_Index), pos_all(N,3,t_Index), 'o', 'MarkerSize', S, 'MarkerFaceColor', color(N,:), 'MarkerEdgeColor', color(N,:))
    end
    drawnow
    movieVector(frameIndex) = getframe(gcf);  % assign frame using consecutive index
    frameIndex = frameIndex + 1;  
end


%% Create movie
% sets up video writer as a .mp4 with 30fps
video_Writer = VideoWriter('ElectroMag_Particle_Sim','MPEG-4');
video_Writer.FrameRate = 30;

% write video
open(video_Writer);
writeVideo(video_Writer, movieVector);
close(video_Writer);