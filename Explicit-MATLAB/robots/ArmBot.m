classdef ArmBot < RobotPrimitive & handle
    methods
        function obj = ArmBot(L1, L2, L3, m1, m2, m3)
            % Defaults
            if nargin < 1, L1 = 0.3; end
            if nargin < 2, L2 = 0.25; end
            if nargin < 3, L3 = 0.15; end
            if nargin < 4, m1 = 2.0; end
            if nargin < 5, m2 = 1.5; end
            if nargin < 6, m3 = 1.2; end

            obj.Name = 'ArmBot';
            obj.Dimension = 3;
            obj.nq = 5;

            % Joint structure
            obj.ParentID = 0:obj.nq - 1;
            obj.JointTypes = ones(1, obj.nq);
            obj.AxisDirections = repmat([0; 0; 1], 1, obj.nq);
            obj.AxisOrigins = zeros(3, obj.nq);

            % Mass assignment
            obj.Masses = [2, 2, m1, m2, m3];  % shoulder mass lumped to joints 1-2

            % Inertias
            obj.Inertias = zeros(6, obj.nq);
            for i = 1:obj.nq
                m = obj.Masses(i);
                obj.Inertias(:, i) = [1 1 1 0 0 0] * m * 0.01;
            end

            % Link transforms
            obj.H_init = repmat(eye(4), 1, 1, obj.nq);
            obj.H_init(1:3, 4, 3) = [0; 0; L1];   % shoulder → elbow
            obj.H_init(1:3, 4, 4) = [0; 0; L2];   % elbow → wrist
            obj.H_init(1:3, 4, 5) = [0; 0; L3];   % wrist → hand

            % COM transforms (roughly halfway along each link)
            obj.H_COM_init = repmat(eye(4), 1, 1, obj.nq);
            obj.H_COM_init(1:3, 4, 3) = [0; 0; L1/2];
            obj.H_COM_init(1:3, 4, 4) = [0; 0; L2/2];
            obj.H_COM_init(1:3, 4, 5) = [0; 0; L3/2];
        end
    end
end