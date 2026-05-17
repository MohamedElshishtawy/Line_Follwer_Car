clear; clc; close all;
% =========================================================================
% 1. ROBOT CAR AND TRACK PARAMETERS
% =========================================================================
rR = 0.0325;         % Radius of Right wheel (meters)
rL = 0.0325;         % Radius of Left wheel (meters)
b  = 0.1;            % Half-width of the robot (meters)
v  = 0.3;            % Target forward velocity (m/s)

% Track dimensions based on the provided image
L_side   = 1.0;      % Length of vertical sides (100cm)
L_bottom = 1.0;      % Length of bottom side (100cm)
R_turn   = 0.5;      % Radius of the top semi-circle (50cm)

% Starting point (Middle of the bottom line)
start_x     = 0.5;   % x-coordinate (exactly in the middle of the 1m line)
start_y     = 0.0;   % y-coordinate
start_theta = 0;     % Initial heading (0 rad = facing right)

% Spin velocity for sharp 90-degree corners (rad/s)
omega_spin = pi/4;   

% =========================================================================
% 2. TIMING AND COMMAND SIGNALS
% =========================================================================
% The track consists of 7 segments for one complete lap:
t_seg = zeros(1, 7);
t_seg(1) = (1.0 - start_x) / v;       % 1. Straight right to bottom-right corner
t_seg(2) = (pi/2) / omega_spin;       % 2. Point turn 90 deg CCW
t_seg(3) = L_side / v;                % 3. Straight up the right side
t_seg(4) = (pi * R_turn) / v;         % 4. Top semi-circle U-turn
t_seg(5) = L_side / v;                % 5. Straight down the left side
t_seg(6) = (pi/2) / omega_spin;       % 6. Point turn 90 deg CCW
t_seg(7) = start_x / v;               % 7. Straight right back to the start mark

t_end = cumsum(t_seg);                % End times for each segment
t_lap = t_end(end);                   % Total time for one complete lap

dt = 0.003;
t = 0:dt:(2 * t_lap);                 % Run for exactly 2 full laps

% Pre-allocate wheel velocities
phidotR = zeros(1, length(t));
phidotL = zeros(1, length(t));

% Fill the arrays with commands based on the track shape
for i = 1:length(t)
    time_in_lap = mod(t(i), t_lap);
    
    if time_in_lap <= t_end(1)
        % Segment 1: Straight right
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(2)
        % Segment 2: Spin 90 deg CCW in place
        phidotR(i) =  omega_spin * b / rR; 
        phidotL(i) = -omega_spin * b / rL;
    elseif time_in_lap <= t_end(3)
        % Segment 3: Straight up
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(4)
        % Segment 4: Top Semi-circle
        omega_curve = v / R_turn;
        phidotR(i) = (v + omega_curve * b) / rR; 
        phidotL(i) = (v - omega_curve * b) / rL;
    elseif time_in_lap <= t_end(5)
        % Segment 5: Straight down
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    elseif time_in_lap <= t_end(6)
        % Segment 6: Spin 90 deg CCW in place
        phidotR(i) =  omega_spin * b / rR; 
        phidotL(i) = -omega_spin * b / rL;
    else
        % Segment 7: Straight right to complete lap
        phidotR(i) = v / rR; 
        phidotL(i) = v / rL;
    end
end

% =========================================================================
% 3. KINEMATICS INTEGRATION
% =========================================================================
x     = zeros(1, length(t));
y     = zeros(1, length(t));
theta = zeros(1, length(t));

% Apply the starting positions
x(1)     = start_x;
y(1)     = start_y;
theta(1) = start_theta;

for ii = 2:length(t)
    J = [rR*cos(theta(ii-1))/2,  rL*cos(theta(ii-1))/2; ...
         rR*sin(theta(ii-1))/2,  rL*sin(theta(ii-1))/2; ...
         0.5*rR/b,              -0.5*rL/b];
    deltapose = dt * J * [phidotR(ii-1); phidotL(ii-1)];
    x(ii)     = x(ii-1)     + deltapose(1);
    y(ii)     = y(ii-1)     + deltapose(2);
    theta(ii) = theta(ii-1) + deltapose(3);
end

% =========================================================================
% 4. VISUALIZATION & ANIMATION
% =========================================================================
figure('Name', 'Window Shaped Track Animation', 'Position', [100, 100, 600, 700]);
hold on; axis equal; grid on;
title('Differential Drive Robot on Window Track');
xlabel('X (meters)'); ylabel('Y (meters)');
xlim([-0.4, 1.4]); ylim([-0.4, 1.8]);

% Draw the new track
plot([0, 1], [0, 0], 'k--', 'LineWidth', 2); % Bottom straight
plot([1, 1], [0, 1], 'k--', 'LineWidth', 2); % Right straight
plot([0, 0], [1, 0], 'k--', 'LineWidth', 2); % Left straight
ang = linspace(0, pi, 50);
plot(0.5 + 0.5*cos(ang), 1 + 0.5*sin(ang), 'k--', 'LineWidth', 2); % Top semi-circle

% Draw the start mark from the middle of the bottom line
plot([start_x start_x], [-0.05 0.05], 'r-', 'LineWidth', 2);
text(start_x, -0.1, 'Start (Middle)', 'Color', 'r', 'HorizontalAlignment', 'center');

% Define robot shape
chassis_x = [-0.1,  0.1,  0.1, -0.1];
chassis_y = [-0.1, -0.1,  0.1,  0.1];
wheel_x   = [-0.08, 0.08, 0.08, -0.08];
wheelR_y  = [-0.17, -0.17, -0.11, -0.11];
wheelL_y  = [ 0.11,  0.11,  0.17,  0.17];

h_chassis = patch('XData', [], 'YData', [], 'FaceColor', [0.7 0.7 0.7], 'EdgeColor', 'k');
h_wheelR  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_wheelL  = patch('XData', [], 'YData', [], 'FaceColor', 'k');
h_nose    = plot(0, 0, 'r-', 'LineWidth', 3);

% Animation loop 
for ii = 1:10:length(t)
    th    = theta(ii);
    R_mat = [cos(th), -sin(th); sin(th), cos(th)];
    
    rot_chassis = R_mat * [chassis_x; chassis_y];
    rot_wheelR  = R_mat * [wheel_x;   wheelR_y];
    rot_wheelL  = R_mat * [wheel_x;   wheelL_y];
    
    set(h_chassis, 'XData', rot_chassis(1,:) + x(ii), 'YData', rot_chassis(2,:) + y(ii));
    set(h_wheelR,  'XData', rot_wheelR(1,:)  + x(ii), 'YData', rot_wheelR(2,:)  + y(ii));
    set(h_wheelL,  'XData', rot_wheelL(1,:)  + x(ii), 'YData', rot_wheelL(2,:)  + y(ii));
    set(h_nose, 'XData', [x(ii), x(ii) + 0.15*cos(th)], 'YData', [y(ii), y(ii) + 0.15*sin(th)]);
    
    drawnow;
end
fprintf('Finished simulating exactly %d points.\n', length(t));