classdef ArmBot < RobotPrimitive & handle
    % ArmBot: 7-DOF model of the human arm
    % Joint 1: Spherical joint (3 DOF) – Shoulder
    % Joint 2: Revolute joint (1 DOF) – Elbow
    % Joint 3: Spherical joint (3 DOF) – Wrist

    methods
        function obj = ArmBot(L1, L2, L3)
            if nargin < 1, L1 = 0.3; end  % Upper arm length
            if nargin < 2, L2 = 0.25; end % Forearm length
            if nargin < 3, L3 = 0.15; end % Hand length

            obj.Name = 'ArmBot';
            obj.Dimension = 3;
            obj.nq = 7;
            obj.ParentID = 0:obj.nq;

            % Masses (estimated for anatomical segments)
            obj.Masses = [2, 2, 2, 1.5, 1.2, 1.2, 1.2];

            % Simple inertia tensors (Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
            obj.Inertias = zeros(6, obj.nq);
            for i = 1:obj.nq
                m = obj.Masses(i);
                obj.Inertias(:,i) = [1 1 1 0 0 0] * m * 0.01;
            end

            % All joints are revolute
            obj.JointTypes = ones(1, obj.nq);

            % --------------------------------------------------
            % Joint Axes and Origins (3+1+3 = 7 revolute joints)
            % --------------------------------------------------
            obj.AxisOrigins = zeros(3, obj.nq);

            % Shoulder (Joint 1: spherical, 3 revolute joints)
            obj.AxisOrigins(:,2) = [0; 0; 0];
            obj.AxisOrigins(:,3) = [0; 0; 0];

            % Elbow (Joint 2: revolute)
            obj.AxisOrigins(:,4) = [0; 0; L1];

            % Wrist (Joint 3: spherical, 3 revolute joints)
            obj.AxisOrigins(:,5) = [0; 0; L1 + 0.01];
            obj.AxisOrigins(:,6) = [0; 0; L1 + L2];
            obj.AxisOrigins(:,7) = [0; 0; L1 + L2 + 0.01];

            % Axis directions (orthogonal rotations)
            obj.AxisDirections = [ 
                0  1  0  0  0  1  0;  % x/y/z/y/z/y/z
                1  0  0  1  0  0  1;
                0  0  1  0  1  0  0 ];

            % --------------------------------------------------
            % Homogeneous Transforms (base to EE)
            % --------------------------------------------------
            obj.H_init = repmat(eye(4), 1, 1, obj.nq + 1);
            obj.H_init(3,4,5) = L1;              % after shoulder
            obj.H_init(3,4,6) = L1 + L2/2;       % mid-forearm
            obj.H_init(3,4,7) = L1 + L2;         % start of hand
            obj.H_init(3,4,8) = L1 + L2 + L3;    % fingertip

            % Center of mass transforms (mid-segment positions)
            obj.H_COM_init = repmat(eye(4), 1, 1, obj.nq);
            obj.H_COM_init(3,4,4) = L1/2;                    % upper arm
            obj.H_COM_init(3,4,5) = L1 + 0.01;               % elbow link
            obj.H_COM_init(3,4,6) = L1 + L2/2;               % forearm
            obj.H_COM_init(3,4,7) = L1 + L2 + L3/2;          % hand

            obj.gMarkerSize = 15 * ones(1, obj.nq);
        end
    end
end