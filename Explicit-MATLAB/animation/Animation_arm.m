classdef Animation_arm < handle
    % 3D Animation class for the ArmBot robot

    properties
        xLim
        yLim
        zLim
        Dimension = 3
        Title = 'ArmBot Simulator'
        t = 0
        isSaveVideo = false
        VideoSpeed = 1.0
        FrameUpdateTime
        nFrame = 0
        hFig
        hAxes
        Robots = {}
        nRobots = 0
        gLinks = {}
        gObjs = {}
        gSpheres = {}
        gCylinders = {}
        colorMap
    end

    methods

        function obj = Animation_arm(varargin)
            p = inputParser;
            addParameter(p, 'xLim', [-0.5, 0.5]);
            addParameter(p, 'yLim', [-0.5, 0.5]);
            addParameter(p, 'zLim', [0, 1.5]);
            addParameter(p, 'Title', 'ArmBot Simulator');
            addParameter(p, 'isSaveVideo', false);
            addParameter(p, 'VideoSpeed', 1.0);
            parse(p, varargin{:});

            obj.xLim = p.Results.xLim;
            obj.yLim = p.Results.yLim;
            obj.zLim = p.Results.zLim;
            obj.Title = p.Results.Title;
            obj.isSaveVideo = p.Results.isSaveVideo;
            obj.VideoSpeed = p.Results.VideoSpeed;

            % Predefined colormap for links
            obj.colorMap = lines(10);  % or distinguishable_colors if available
        end

        function init(obj)
            obj.FrameUpdateTime = 1.0 / 30 * obj.VideoSpeed;
            obj.hFig = figure('Color', [1,1,1], 'Name', obj.Title);
            obj.hAxes = axes('Parent', obj.hFig);
            axis equal; axis manual; grid on; hold on;
            xlabel('x'); ylabel('y'); zlabel('z');
            xlim(obj.xLim); ylim(obj.yLim); zlim(obj.zLim);
            view(135, 25);
            camlight('headlight'); lighting gouraud;
        end

        function attachRobot(obj, robot)
            obj.nRobots = obj.nRobots + 1;
            n = obj.nRobots;
            obj.Robots{n} = robot;
            nq = robot.nq;
            obj.gSpheres{n} = gobjects(1, nq);
            obj.gCylinders{n} = gobjects(1, nq - 1);  % only nq-1 segments between nq joints

            % Draw joints as spheres and bones as cylinders
            for j = 1:nq-1
                p1 = robot.H_ij(1:3,4,j);
                p2 = robot.H_ij(1:3,4,j+1);

                % Sphere for joint
                [xs, ys, zs] = sphere(10);
                r = 0.015;
                obj.gSpheres{n}(i) = surf(r*xs + p1(1), r*ys + p1(2), r*zs + p1(3), ...
                    'FaceColor', 'b', 'EdgeColor', 'none');

                % Cylinder for link with color
                color = obj.colorMap(mod(i-1, size(obj.colorMap,1))+1, :);
                [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                obj.gCylinders{n}(i) = surf(Xc, Yc, Zc, 'FaceColor', color, 'EdgeColor', 'none');
            end
        end

        function update(obj, t)
            obj.t = t;
            for i = 1:obj.nRobots
                robot = obj.Robots{i};
                nq = robot.nq;
                for j = 1:nq
                    p1 = robot.H_ij(1:3,4,j);
                    if j < nq
                        p2 = robot.H_ij(1:3,4,j+1);
                    else
                        p2 = robot.getForwardKinematics(robot.q);
                        p2 = p2(1:3,4);
                    end

                    % Update sphere (joint)
                    [xs, ys, zs] = sphere(10);
                    r = 0.015;
                    set(obj.gSpheres{i}(j), 'XData', r*xs + p1(1), 'YData', r*ys + p1(2), 'ZData', r*zs + p1(3));

                    % Update cylinder (bone)
                    [Xc, Yc, Zc] = createCylinder(p1, p2, 0.01, 10);
                    set(obj.gCylinders{i}(j), 'XData', Xc, 'YData', Yc, 'ZData', Zc);
                end
            end
            drawnow;
        end

    end
end

function [X, Y, Z] = createCylinder(p1, p2, radius, n)
% Creates a cylinder from p1 to p2 with given radius and resolution n
[Xc, Yc, Zc] = cylinder(radius, n);
v = p2 - p1;
h = norm(v);
Zc = Zc * h;

% Orthonormal basis
v = v / h;
if abs(dot(v, [0 0 1])) > 0.99
    u = cross(v, [1 0 0]);
else
    u = cross(v, [0 0 1]);
end
u = u / norm(u);
w = cross(v, u);

R = [u(:), w(:), v(:)];
pts = R * [Xc(:)'; Yc(:)'; Zc(:)'];
X = reshape(pts(1,:) + p1(1), size(Xc));
Y = reshape(pts(2,:) + p1(2), size(Yc));
Z = reshape(pts(3,:) + p1(3), size(Zc));
end