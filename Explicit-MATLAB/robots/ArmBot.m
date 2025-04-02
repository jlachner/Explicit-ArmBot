classdef ArmBot < RobotPrimitive & handle
    % ArmBot: 7-DOF anatomically structured model of the human arm
    % - Joints 1–3: Shoulder (spherical joint via 3 serial revolute joints: X, Y, Z)
    % - Joint 4: Elbow (hinge)
    % - Joints 5–7: Wrist (spherical joint via 3 serial revolute joints: X, Y, Z)

    methods
        function obj = ArmBot(L1, L2, L3, m1, m2, m3)
            % Default values
            if nargin < 1, L1 = 0.3; end     % Upper arm length
            if nargin < 2, L2 = 0.25; end    % Forearm length
            if nargin < 3, L3 = 0.15; end    % Hand length
            if nargin < 4, m1 = 2.0; end     % Upper arm mass
            if nargin < 5, m2 = 1.5; end     % Forearm mass
            if nargin < 6, m3 = 1.2; end     % Hand/wrist mass

            obj.Name = 'ArmBot';
            obj.Dimension = 3;
            obj.nq = 7;

            % Joint hierarchy (0 = base)
            obj.ParentID = 0:obj.nq - 1;
            obj.JointTypes = ones(1, obj.nq);

            % Assign orthogonal axes for shoulder (1–3) and wrist (5–7)
            obj.AxisDirections = zeros(3, obj.nq);
            obj.AxisDirections(:,1) = [1; 0; 0];  % Shoulder DOF 1
            obj.AxisDirections(:,2) = [0; 1; 0];  % Shoulder DOF 2
            obj.AxisDirections(:,3) = [0; 0; 1];  % Shoulder DOF 3
            obj.AxisDirections(:,4) = [0; 1; 0];  % Elbow (hinge)
            obj.AxisDirections(:,5) = [1; 0; 0];  % Wrist DOF 1
            obj.AxisDirections(:,6) = [0; 1; 0];  % Wrist DOF 2
            obj.AxisDirections(:,7) = [0; 0; 1];  % Wrist DOF 3

            obj.AxisOrigins = zeros(3, obj.nq);

            % Mass distribution
            shoulder_mech_mass = 1.0;
            obj.Masses = [ ...
                shoulder_mech_mass, ...
                shoulder_mech_mass, ...
                m1, ...          % upper arm
                m2, ...          % forearm
                m3/3, m3/3, m3/3 % wrist
            ];

            % Inertia tensors (simple approximation)
            obj.Inertias = zeros(6, obj.nq);
            for i = 1:obj.nq
                m = obj.Masses(i);
                obj.Inertias(:, i) = [1 1 1 0 0 0] * m * 0.01;
            end

            % Joint transforms: only apply link lengths at joints 3, 4, 7
            obj.H_init = repmat(eye(4), 1, 1, obj.nq);
            obj.H_init(1:3, 4, 3) = [0; 0; L1];   % Shoulder → Elbow
            obj.H_init(1:3, 4, 4) = [0; 0; L2];   % Elbow → Wrist
            obj.H_init(1:3, 4, 7) = [0; 0; L3];   % Wrist → Hand

            % Center of mass transforms
            obj.H_COM_init = repmat(eye(4), 1, 1, obj.nq);
            obj.H_COM_init(1:3, 4, 3) = [0; 0; L1 / 2];
            obj.H_COM_init(1:3, 4, 4) = [0; 0; L2 / 2];
            obj.H_COM_init(1:3, 4, 7) = [0; 0; L3 / 2];
        end
    end
end