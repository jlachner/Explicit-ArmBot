classdef Animation_arm < handle
    properties
        Figure
        Axes
        Robots
        nRobots
        gSpheres
        gCylinders

        xLim
        yLim
        zLim
    end

    methods
        function obj = Animation_arm(varargin)
            obj.Robots = {};
            obj.nRobots = 0;

            % Parse input arguments for axis limits
            p = inputParser;
            addParameter(p, 'xLim', [0 1]);
            addParameter(p, 'yLim', [0 1]);
            addParameter(p, 'zLim', [0 1]);
            parse(p, varargin{:});
            obj.xLim = p.Results.xLim;
            obj.yLim = p.Results.yLim;
            obj.zLim = p.Results.zLim;

            % Create figure and axes
            obj.Figure = figure('Color', 'w');
            obj.Axes = axes('Parent', obj.Figure);
            xlabel(obj.Axes, 'X'); ylabel(obj.Axes, 'Y'); zlabel(obj.Axes, 'Z');
            hold(obj.Axes, 'on');
            grid(obj.Axes, 'on');
            view(obj.Axes, 135, 20);
            camproj(obj.Axes, 'perspective');
            camup(obj.Axes, [0 0 1]);
            axis(obj.Axes, 'equal');

            obj.gSpheres = {};
            obj.gCylinders = {};
        end

        function init(obj)
            cla(obj.Axes);
            hold(obj.Axes, 'on');
            grid(obj.Axes, 'on');
            axis(obj.Axes, 'equal');
            xlim(obj.Axes, obj.xLim);
            ylim(obj.Axes, obj.yLim);
            zlim(obj.Axes, obj.zLim);
            axis(obj.Axes, 'manual');
        end

        function attachRobot(obj, robot)
            obj.nRobots = obj.nRobots + 1;
            obj.Robots{obj.nRobots} = robot;

            nq = robot.nq;
            obj.gSpheres{obj.nRobots} = gobjects(nq + 1, 1);  % +1 for shoulder origin
            obj.gCylinders{obj.nRobots} = gobjects(nq, 1);

            T = repmat(eye(4), 1, 1, nq + 1);
            T(:, :, 1) = eye(4);
            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                T(:, :, j + 1) = T(:, :, parent) * robot.H_ij(:, :, j) * robot.H_init(:, :, j);
            end

            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                p1 = T(1:3, 4, parent);
                p2 = T(1:3, 4, j + 1);

                % Link color
                if j <= 3
                    linkColor = [1.0 0.5 0.1];  % shoulder (orange)
                elseif j == 4
                    linkColor = [0.5 0.3 0.9];  % elbow (purple)
                else
                    linkColor = [0 0 0];        % wrist (black)
                end

                % Sphere color logic
                if j + 1 == 2
                    sphereColor = [0.5 0.3 0.9];  % purple
                else
                    sphereColor = linkColor;
                end

                % --- DEBUG PRINTS ---
                fprintf('Joint %d:\n', j + 1);
                fprintf('  Parent ID: %d\n', parent);
                fprintf('  p1 = [%f, %f, %f]\n', p1);
                fprintf('  p2 = [%f, %f, %f]\n', p2);
                fprintf('  linkColor = [%f %f %f]\n', linkColor);
                fprintf('  sphereColor = [%f %f %f]\n', sphereColor);
                fprintf('\n');

                % Draw link
                [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                obj.gCylinders{obj.nRobots}(j) = surf(Xc, Yc, Zc, ...
                    'FaceColor', linkColor, 'EdgeColor', 'none');

                % Draw sphere at joint j+1
                [xs, ys, zs] = sphere(10);
                r = 0.015;
                obj.gSpheres{obj.nRobots}(j + 1) = surf(r * xs + p2(1), r * ys + p2(2), r * zs + p2(3), ...
                    'FaceColor', sphereColor, 'EdgeColor', 'none');
            end

            % Add shoulder base sphere
            p0 = T(1:3, 4, 1);
            [xs, ys, zs] = sphere(10);
            r = 0.015;
            obj.gSpheres{obj.nRobots}(1) = surf(r * xs + p0(1), r * ys + p0(2), r * zs + p0(3), ...
                'FaceColor', [1.0 0.5 0.1], 'EdgeColor', 'none');

            % --- DEBUG PRINT FOR BASE SPHERE ---
            fprintf('Base sphere (joint 1):\n');
            fprintf('  p0 = [%f, %f, %f]\n', p0);
            fprintf('  color = [1.000 0.500 0.100] (orange)\n\n');
        end

        function update(obj, t)
            for i = 1:obj.nRobots
                robot = obj.Robots{i};
                nq = robot.nq;

                T = repmat(eye(4), 1, 1, nq + 1);
                T(:, :, 1) = eye(4);
                for j = 1:nq
                    parent = robot.ParentID(j) + 1;
                    T(:, :, j + 1) = T(:, :, parent) * robot.H_ij(:, :, j) * robot.H_init(:, :, j);
                end

                for j = 1:nq
                    parent = robot.ParentID(j) + 1;
                    p1 = T(1:3, 4, parent);
                    p2 = T(1:3, 4, j + 1);

                    if isvalid(obj.gCylinders{i}(j))
                        [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                        set(obj.gCylinders{i}(j), 'XData', Xc, 'YData', Yc, 'ZData', Zc);
                    end

                    if isvalid(obj.gSpheres{i}(j + 1))
                        [xs, ys, zs] = sphere(10);
                        r = 0.015;
                        set(obj.gSpheres{i}(j + 1), ...
                            'XData', r * xs + p2(1), ...
                            'YData', r * ys + p2(2), ...
                            'ZData', r * zs + p2(3));
                    end
                end

                % Update shoulder sphere at base
                p0 = T(1:3, 4, 1);
                if isvalid(obj.gSpheres{i}(1))
                    [xs, ys, zs] = sphere(10);
                    r = 0.015;
                    set(obj.gSpheres{i}(1), ...
                        'XData', r * xs + p0(1), ...
                        'YData', r * ys + p0(2), ...
                        'ZData', r * zs + p0(3));
                end
            end
            drawnow limitrate nocallbacks;
        end
    end
end

function [Xc, Yc, Zc] = createCylinder(p1, p2, radius, resolution)
v = p2 - p1;
height = norm(v);
[X, Y, Z] = cylinder(radius, resolution);
Z = Z * height;

z = [0 0 1]';
if norm(cross(z, v)) == 0
    R = eye(3);
else
    v = v / norm(v);
    axis = cross(z, v);
    axis = axis / norm(axis);
    theta = acos(dot(z, v));
    K = [0 -axis(3) axis(2); axis(3) 0 -axis(1); -axis(2) axis(1) 0];
    R = eye(3) + sin(theta)*K + (1 - cos(theta)) * K^2;
end

XYZ = R * [X(:)'; Y(:)'; Z(:)'];
Xc = reshape(XYZ(1, :), size(X)) + p1(1);
Yc = reshape(XYZ(2, :), size(Y)) + p1(2);
Zc = reshape(XYZ(3, :), size(Z)) + p1(3);
end