% main_arm_sim.m
% Simulates a 3D human arm model using ArmBot and Animation_arm

clear; close all; clc;

% Time settings
dt = 0.05;
t_total = 5;
t = 0:dt:t_total;

% Create the robot and initialize
robot = ArmBot();
robot.init();

% Initial joint positions (in radians)
q = deg2rad([30, -20, 10, 45, -15, 20, 10])';
dq = zeros(robot.nq,1);
robot.updateKinematics(q);

% Create and initialize animation
anim = Animation_arm('xLim', [-0.5 0.5], 'yLim', [-0.5 0.5], 'zLim', [0 1.2]);
anim.init();
anim.attachRobot(robot);

% Animation loop
for i = 1:length(t)
    % Simulate a small oscillation at the wrist
    q(6) = deg2rad(20) * sin(2*pi*0.5*t(i));
    q(7) = deg2rad(15) * cos(2*pi*0.5*t(i));
    
    robot.updateKinematics(q);
    anim.update(t(i));
end
