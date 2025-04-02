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
            obj.gSpheres{obj.nRobots} = cell(nq + 1, 1);  
            obj.gCylinders{obj.nRobots} = gobjects(nq, 1);

            T = repmat(eye(4), 1, 1, nq + 1);
            T(:, :, 1) = eye(4);
            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                T(:, :, j + 1) = T(:, :, parent) * robot.H_ij(:, :, j) * robot.H_init(:, :, j);
            end

            [xs, ys, zs] = sphere(10);
            r = 0.015;

            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                p1 = T(1:3, 4, parent);
                p2 = T(1:3, 4, j + 1);

                % Assign color based on link index
                if j <= 3
                    color = [1.0 0.5 0.1];  % shoulder
                elseif j == 4
                    color = [0.5 0.3 0.9];  % elbow
                else
                    color = [0 0 0];        % wrist
                end

                % Draw link
                [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                obj.gCylinders{obj.nRobots}(j) = surf(Xc, Yc, Zc, ...
                    'FaceColor', color, 'EdgeColor', 'none');

                % Draw sphere at p1
                obj.gSpheres{obj.nRobots}{parent} = surf( ...
                    r * xs + p1(1), r * ys + p1(2), r * zs + p1(3), ...
                    'FaceColor', color, 'EdgeColor', 'none');
            end
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

                [xs, ys, zs] = sphere(10);
                r = 0.015;

                for j = 1:nq
                    parent = robot.ParentID(j) + 1;
                    p1 = T(1:3, 4, parent);
                    p2 = T(1:3, 4, j + 1);

                    % Update link
                    if isvalid(obj.gCylinders{i}(j))
                        [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                        set(obj.gCylinders{i}(j), 'XData', Xc, 'YData', Yc, 'ZData', Zc);
                    end

                    % Update sphere at p1
                    if ~isempty(obj.gSpheres{i}{parent}) && isvalid(obj.gSpheres{i}{parent})
                        set(obj.gSpheres{i}{parent}, ...
                            'XData', r * xs + p1(1), ...
                            'YData', r * ys + p1(2), ...
                            'ZData', r * zs + p1(3));
                    end
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