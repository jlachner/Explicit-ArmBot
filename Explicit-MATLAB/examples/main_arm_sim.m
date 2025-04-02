% Clean-up
clear; close all; clc;

% Time settings
dt = 0.05;
t_total = 50;
t = 0:dt:t_total;

% Load subject data from Excel
data = readtable('../subject_data.xlsx');

% Choose which subject to simulate
subject_id = "S02";  
s = find(data.SubjectID == subject_id);
body_mass = data.Weight_kg(s);
l1 = data.UpperArm_m(s);
l2 = data.Forearm_m(s);
l3 = data.Hand_m(s);
sex = data.Sex{s}; 

% Compute segment properties based on Winter (1990)
[segment_mass, segment_I] = compute_segment_properties(body_mass, [l1, l2, l3], sex);

% Display subject summary — this MUST be outside the function
fprintf('\n--- Subject Summary ---\n');
fprintf('Subject ID   : %s\n', string(data.SubjectID(s)));
fprintf('Sex          : %s\n', string(sex));
fprintf('Age          : %d years\n', data.Age(s));
fprintf('Handedness   : %s\n', string(data.Handedness(s)));
fprintf('Height       : %.2f m\n', data.Height_m(s));
fprintf('Weight       : %.1f kg\n', body_mass);
fprintf('Upper Arm    : %.2f m, %.2f kg, I = %.4f\n', l1, segment_mass(1), segment_I(1));
fprintf('Forearm      : %.2f m, %.2f kg, I = %.4f\n', l2, segment_mass(2), segment_I(2));
fprintf('Hand         : %.2f m, %.2f kg, I = %.4f\n', l3, segment_mass(3), segment_I(3));
fprintf('------------------------\n\n');

% Initialize robot with segment lengths only
arm = ArmBot(l1, l2, l3);
arm.setMasses(segment_mass);
arm.setInertias(segment_I);
arm.init();

% Fixed joint configuration
q = deg2rad([0, 0, 0, 0, 0, 0, 0])';
arm.updateKinematics(q);   

% Create animation
anim = Animation_arm('xLim', [-0.5 0.5], 'yLim', [-0.5 0.5], 'zLim', [-0.2 1.0]);
anim.init();
anim.attachRobot(arm);    

% Animation loop — only move the wrist joint (joint 5)
for i = 1:length(t)
    q_temp = q;
    q_temp(5) = deg2rad(15) * sin(2 * pi * 0.5 * t(i));  % 0.5 Hz oscillation

    H = arm.getForwardKinematics(q_temp);
    %disp("Transformation matrix: ")
    %disp(H)

    J = arm.getHybridJacobian(q_temp);
    %disp("Jacobian matrix: ")
    %disp(J)

    M = arm.getMassMatrix(q_temp);
    %disp("Mass matrix: ")
    %disp(M)

    arm.updateKinematics(q_temp);
    anim.update(t(i));
end


%% Function to calculate the segment properties
function [segment_mass, segment_I] = compute_segment_properties(body_mass, segment_lengths, sex)
% Based on Winter's anthropometry: mass % and radius of gyration % per segment
% Order: Upper Arm, Forearm, Hand

% Winter's anthropometric data (mass % and radius of gyration %)
switch upper(sex)
    case 'M'
        mass_percent = [0.028, 0.016, 0.006];  % [Upper Arm, Forearm, Hand]
        k_percent =   [0.322, 0.303, 0.297];
    case 'F'
        mass_percent = [0.025, 0.013, 0.005];  % Females have slightly lower segment %s
        k_percent =   [0.328, 0.310, 0.305];   % Females have slightly higher k
    otherwise
        warning('Unknown sex. Defaulting to male parameters.');
        mass_percent = [0.028, 0.016, 0.006];
        k_percent =   [0.322, 0.303, 0.297];
end

segment_mass = mass_percent * body_mass;
segment_I = zeros(3, 1);  % scalar moment of inertia for each segment

for i = 1:3
    l = segment_lengths(i);
    m = segment_mass(i);
    k = k_percent(i) * l;
    segment_I(i) = m * k^2;  % moment of inertia about CoM, transverse axis
end

end