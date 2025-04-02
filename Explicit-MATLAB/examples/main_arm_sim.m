% main_arm_sim.m
% Simulates a 3D human arm model using ArmBot and Animation_arm

clear; close all; clc;

% Time settings
dt = 0.05;
t_total = 50;
t = 0:dt:t_total;

% Create the robot and initialize
l1 = 0.35;
l2 = 0.28;
l3 = 0.18;
m1 = 2.2;
m2 = 1.6;
m3 = 1.0;
robot = ArmBot(l1, l2, l3, m1, m2, m3);
robot.init();

% Fixed joint configuration
q = deg2rad([0, 0, 0, 0, 0, 0, 0])';
robot.updateKinematics(q);   

% Create animation
anim = Animation_arm('xLim', [-0.5 0.5], 'yLim', [-0.5 0.5], 'zLim', [-0.2 1.0]);
anim.init();
anim.attachRobot(robot);     

% Animation loop — only move the wrist joint (joint 5)
for i = 1:length(t)
    q_temp = q;
    q_temp(5) = deg2rad(15) * sin(2 * pi * 0.5 * t(i));  % 0.5 Hz oscillation
    robot.updateKinematics(q_temp);
    anim.update(t(i));
end