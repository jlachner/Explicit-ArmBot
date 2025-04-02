classdef Animation_arm < handle
    properties
        fig
        ax
        Robots
        gSpheres
        gCylinders
        nRobots = 0
        colorMap
    end

    methods
        function obj = Animation_arm(varargin)
            obj.fig = figure('Color','w');
            obj.ax = axes(obj.fig);
            axis(obj.ax, 'equal');
            view(obj.ax, 3);
            grid(obj.ax, 'on');
            xlabel(obj.ax, 'X');
            ylabel(obj.ax, 'Y');
            zlabel(obj.ax, 'Z');
            obj.colorMap = lines(7);
            set(obj.ax, varargin{:});
        end

        function init(obj)
            cla(obj.ax);
            hold(obj.ax, 'on');
        end

        function attachRobot(obj, robot)
            obj.nRobots = obj.nRobots + 1;
            n = obj.nRobots;
            obj.Robots{n} = robot;
            nq = robot.nq;

            obj.gSpheres{n} = gobjects(1, nq);
            obj.gCylinders{n} = gobjects(1, nq);

            T = repmat(eye(4), 1, 1, nq + 1);
            T(:,:,1) = eye(4);

            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                T(:,:,j+1) = T(:,:,parent) * robot.H_ij(:,:,j) * robot.H_init(:,:,j);
            end

            for j = 1:nq
                parent = robot.ParentID(j) + 1;
                p = T(1:3, 4, j+1);

                % Sphere color based on anatomical joint
                if j <= 2
                    joint_color = [0, 0.6, 0.6];    % shoulder
                elseif j == 3
                    joint_color = [1.0, 0.5, 0.1];    % elbow
                elseif j == 4
                    joint_color = [0.5, 0.3, 0.9];    % wrist
                else
                    joint_color = [];         % no sphere for hand
                end

                if ~isempty(joint_color)
                    [xs, ys, zs] = sphere(10);
                    r = 0.015;
                    obj.gSpheres{n}(j) = surf(obj.ax, r*xs + p(1), r*ys + p(2), r*zs + p(3), ...
                                              'FaceColor', joint_color, 'EdgeColor', 'none');
                end

                % Link color based on body segment being drawn
                if robot.ParentID(j) >= 0
                    p1 = T(1:3, 4, parent);
                    p2 = p;

                    if j == 3
                        link_color = [0, 0.6, 0.6];   % shoulder → elbow
                    elseif j == 4
                        link_color = [1.0, 0.5, 0.1];   % elbow → wrist 
                    elseif j == 5
                        link_color = [0.5, 0.3, 0.9];   % wrist → hand
                    else
                        link_color = [0.5 0.5 0.5];  % gray fallback
                    end

                    [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                    obj.gCylinders{n}(j) = surf(obj.ax, Xc, Yc, Zc, ...
                                                'FaceColor', link_color, 'EdgeColor', 'none');
                end
            end
        end

        function update(obj, t)
            for i = 1:obj.nRobots
                robot = obj.Robots{i};
                nq = robot.nq;

                T = repmat(eye(4), 1, 1, nq + 1);
                T(:,:,1) = eye(4);

                for j = 1:nq
                    parent = robot.ParentID(j) + 1;
                    T(:,:,j+1) = T(:,:,parent) * robot.H_ij(:,:,j) * robot.H_init(:,:,j);
                end

                for j = 1:nq
                    parent = robot.ParentID(j) + 1;
                    p = T(1:3, 4, j+1);

                    if isvalid(obj.gSpheres{1}(j))
                        [xs, ys, zs] = sphere(10);
                        r = 0.015;
                        set(obj.gSpheres{1}(j), 'XData', r*xs + p(1), ...
                                                'YData', r*ys + p(2), ...
                                                'ZData', r*zs + p(3));
                    end

                    if robot.ParentID(j) >= 0 && isvalid(obj.gCylinders{1}(j))
                        p1 = T(1:3, 4, parent);
                        p2 = p;
                        [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                        set(obj.gCylinders{1}(j), 'XData', Xc, ...
                                                  'YData', Yc, ...
                                                  'ZData', Zc);
                    end
                end
            end
            drawnow;
        end
    end
end

function [X, Y, Z] = createCylinder(p1, p2, radius, n)
    if nargin < 4
        n = 20;
    end

    v = p2 - p1;
    height = norm(v);
    if height < 1e-6
        X = []; Y = []; Z = [];
        return;
    end

    [x, y, z] = cylinder(radius, n);
    z = z * height;

    [~, ~, V] = svd([v(:)'; null(v(:)')']);
    R = V';
    pts = R * [x(:) y(:) z(:)]';
    x = reshape(pts(1, :), size(x)) + p1(1);
    y = reshape(pts(2, :), size(y)) + p1(2);
    z = reshape(pts(3, :), size(z)) + p1(3);

    X = x; Y = y; Z = z;
end