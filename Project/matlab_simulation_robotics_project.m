% MATLAB: Simulation Project
%   -Assign dimensions of the links
%   -Create a desired trajectory (within the robot's workspace)
%   -Simulate desired trajectory
%   -Plot Desired and actual Trajectory

% 1. Define the robot's Dimensions
L1 = 3; L2 = 2; L3 = 2; L4 = 3; L5 = 4; L7 = 1; % Length's of each link

% Defining the range for L6 (Variable Prismatic Joint)
L6_min = 1; % min length for L6 link
L6_max = 5; % max length for L6 link

% Define the Desired Trajectory of the HAND
h = 0.01; % Sample time
t = 0:h:30; % t is a vector from 0-30 secs at a sample time of 0.01 sec.

% Position Vector for the End-Effector
px = 7 * ones(size(t)); % ones array the same size as t * 7 = array of 7 the same size as t vector
py = 2 * ones(size(t)); % array of 2 the same size as the t vector
pz = linspace(L6_min+6, L6_max+6, length(t)); % linspace = generate length t points from L6_min --> L6_max 
                                              % 6 added to min and max to compensate for the z height of L4, L3 and L7
%NOA: Identity Matrix Shaped (no orientation change)
nx = ones(size(t));
ny = zeros(size(t));
nz = zeros(size(t));
ox = zeros(size(t));
oy = ones(size(t));
oz = zeros(size(t));
ax = zeros(size(t));
ay = zeros(size(t));
az = ones(size(t));

% Apply the Inverse Kinematics: theta1, theta2, theta3, solve for L6
theta1 = atan2d(ay, ax);
theta2 = atan2d( -cosd(theta1).*ax - sind(theta1).*ay, az );
theta3 = atan2d( sind(theta1).*nx - cosd(theta1).*ny, sind(theta1).*ox - cosd(theta1).*oy ); 
L6 = ((pz - L4 - L5 .* sind(theta2)) ./ cosd(theta2)) - L7;

% Forward Kinematics: Actual path of the robot
px_actual = L5 .* cosd(theta1) .* cosd(theta2) - cosd(theta1) .* sind(theta2) .* (L6 + L7)+3;
py_actual = L5 .* cosd(theta2) .* sind(theta1) - sind(theta1) .* sind(theta2) .* (L6 + L7)+2; 
pz_actual = L4 + cosd(theta2) .* (L6 + L7) + L5 .* sind(theta2);

% Plot the Desired and Actual Trajectories (used this plot for testing)
plot(px, pz, px_actual, pz_actual); drawnow;
legend('Desired','Actual');
title('Plot of Desired/Actual Trajectories for the robot from 2.40');
grid;
% figure;
% plot3(px, py, pz, 'r', 'LineWidth', 2); % Desired Trajectory in red
% hold on;
% plot3(px_actual, py_actual, pz_actual, 'b--', 'LineWidth', 2); % Actual Trajectory in blue dashed line
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% legend('Desired', 'Actual');
% title('Plot of Desired/Actual Trajectories for the robot');
% grid on;
% hold off;
