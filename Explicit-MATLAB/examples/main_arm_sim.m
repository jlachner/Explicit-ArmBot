% main_arm_sim.m
% Simulates a 3D human arm model using ArmBot and Animation_arm

clear; close all; clc;

% Time settings
dt = 0.05;
t_total = 5;
t = 0:dt:t_total;

% Create the robot and initialize
L1 = 0.35;
L2 = 0.28;
L3 = 0.18;
m1 = 2.2;
m2 = 1.6;
m3 = 1.0;
robot = ArmBot(L1, L2, L3, m1, m2, m3);
robot.init();

% Initial joint positions (in radians)
q = deg2rad([30, -20, 10, 45, -15])';
dq = zeros(robot.nq, 1);

% Update kinematics
robot.updateKinematics(q);

% Create and initialize animation
anim = Animation_arm('xLim', [-0.5 0.5], 'yLim', [-0.5 0.5], 'zLim', [-0.2 1.0]);
anim.init();

% Attach robot to animation
anim.attachRobot(robot);

% Animation loop (disabled for now)
% for i = 1:length(t)
%     robot.updateKinematics(q);
%     anim.update(t(i));
% end