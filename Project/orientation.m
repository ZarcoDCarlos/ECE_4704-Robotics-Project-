%Matlab Simulation Project while attempting to change the orientation and
%the position of the hand!
% MATLAB: Simulation Project
%   -Assign dimensions of the links
%   -Create a desired trajectory (within the robot's workspace)
%   -Simulate desired trajectory
%   -Plot Desired and actual Trajectory
%   
% Note: I did not get it working correctly, but this is what I was able to
% do


% 1. Define the robot's Dimensions
L1 = 3; L2 = 2; L3 = 2; L4 = 3; L5 = 4; L7 = 1; % Length's of each link

% Defining the range for L6 (Variable Prismatic Joint)
L6_min = 1;
L6_max = 5;


% Define the Desired Trajectory of the HAND
h = 0.01; % Sample time
t = 0:h:30; % t is a vector from 0-30 secs at a sample time of 0.01 sec.

xo=3;yo=2; %xo,yo is the circle's origin in the plane

x=xo+4*sin(pi/2*t); %Definition of the circle
y=yo+4*cos(pi/2*t); %centered at (xo,yo) with radius 4 

xmid_point = floor(length(t) / 2); %halfway point of vector for px
xdescend = linspace(7, 0, xmid_point); % lower half of x values
xascend = linspace(0, 7, length(t) - xmid_point); %higher half of x values
% to create y vector, I need the y vector to be from 2--> 6--> 2--> -2--> 2
% (these are the values of y around the circle)
seg_len = floor(length(t) / 4); %split y vector into 1/4 length of t
seg1 = linspace(2, 6, seg_len); % segment range from 2-6
seg2 = linspace(6, 2, seg_len); % segment range from 6-0
seg3 = linspace(2, -2, seg_len); % segment range from 0-2
seg4 = linspace(-2, 2, length(t) - 3 * seg_len); % Adjust last segment to ensure total length matches

% Concatenate the segments of x and y to make the position vectors
px = [xdescend, xascend];
py = [seg1, seg2, seg3, seg4];
pz = linspace(L6_min+6, L6_max+6, length(t)); % linspace = generate length t points from L6_min --> L6_max


%Inverse Kinematics: Do not know how to define noa values for orientation
%change
theta1 = atan2d(ay, ax); % 
theta2 = atan2d( -cosd(theta1).*ax - sind(theta1).*ay, az ); % theta 2 = 0
theta3 = atan2d( sind(theta1).*nx - cosd(theta1).*ny, sind(theta1).*ox - cosd(theta1).*oy ); %=180
L6 = ((pz - L4 - L5 .* sind(theta2)) ./ cosd(theta2)) - L7;

% %FORWARD KINEMATICS
x_actual = L5.*cosd(thetal) .*cosd(theta2) - cosd(thetal) .*sind(theta2)*(L6 + L7);
y_actual = L5.*cosd(theta2) .*sind(thetal) - sind(thetal) .*sind(theta2)*(L6 + L7);
z_actual = L4 + cosd(theta2).*(L6 + L7) + L5.*sind(theta2);

 plot(x, y); drawnow;
%legend('Desired','Actual');
title('Plot of Desired/Actual Trajectories for the robot from 2.40');
grid;

