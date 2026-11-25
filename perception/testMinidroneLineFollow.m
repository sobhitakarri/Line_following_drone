function testMinidroneLineFollow()
%TESTMINIDRONELINEFOLLOW Checks the guidance function without Simulink.
% Frames come from renderTrackCamera, so they use the same camera model
% (38 deg HFOV, 0.1 m mount offset) as the Unreal camera.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));

failures = 0;
failures = failures + checkTakeoff();
failures = failures + checkStraight();
failures = failures + checkTurn('right turn yaws clockwise', [0 0; 1.0 0; 1.0 1.0], +1);
failures = failures + checkTurn('left turn yaws counter-clockwise', [0 0; 1.0 0; 1.0 -1.0], -1);
failures = failures + checkSeparatedDisk();
failures = failures + checkYellowIgnored();
failures = failures + checkTrackView();

if failures == 0
    fprintf('minidroneLineFollow: all checks passed.\n');
else
    error('minidroneLineFollow: %d check(s) failed.', failures);
end
end

function n = checkTakeoff()
clear minidroneLineFollow
t = track([0 0; 2 0], [2.2 0 0.1]);
[R, G, B] = renderTrackCamera(0, 0, 0.002, 0, t);
wpt = minidroneLineFollow(R, G, B, [0; 0; 0.002; 0]);
ok = abs(wpt(1)) < 1e-9 && abs(wpt(2)) < 1e-9 && wpt(3) < -1 && abs(wpt(4)) < 1e-9;
n = report('takeoff holds XY and yaw and climbs', ok, wpt);
end

function n = checkStraight()
clear minidroneLineFollow
t = track([0 0; 2 0], [2.2 0 0.1]);
pos = [0.5; 0; -1.1; 0];
wpt = run(pos, t, 3);
[wx, wy] = toWorld(wpt);
ok = wx > pos(1) + 0.1 && abs(wy) < 0.05 && abs(wpt(4)) < 0.05;
n = report('straight line flies forward, no yaw', ok, wpt);
end

function n = checkTurn(name, pts, sgn)
clear minidroneLineFollow
t = track(pts, [pts(end, 1) pts(end, 2) + 0.2 * sgn 0.1]);
pos = [0.85; 0; -1.1; 0];
wpt = run(pos, t, 3);
ok = sgn * wpt(4) > 0.3;
n = report(name, ok, wpt);
end

function n = checkSeparatedDisk()
% Stock layout: circle 0.1 m beyond the line end. Hovering over the line end
% it must head for the circle, not stop or turn back.
clear minidroneLineFollow
t = track([0 0; 2 0], [2.2 0 0.1]);
pos = [1.9; 0; -1.1; 0];
wpt = run(pos, t, 3);
[wx, ~] = toWorld(wpt);
ok1 = wx > 1.95;
% Directly above the circle it must descend and hold the centre.
pos = [2.2; 0; -1.1; 0];
for k = 1:4
    [R, G, B] = renderTrackCamera(pos(1), pos(2), pos(3), pos(4), t);
    wpt = minidroneLineFollow(R, G, B, pos);
end
[wx, wy] = toWorld(wpt);
ok2 = wpt(3) > pos(3) && hypot(wx - 2.2, wy) < 0.03;
n = report('separated circle: approach then descend on centre', ok1 && ok2, wpt);
end

function n = checkYellowIgnored()
clear minidroneLineFollow
cam = minidroneCameraModel();
R = 60 * ones(cam.rows, cam.cols, 'uint8');
G = 90 * ones(cam.rows, cam.cols, 'uint8');
B = 70 * ones(cam.rows, cam.cols, 'uint8');
R(20:100, 40:120) = 220;
G(20:100, 40:120) = 210;
B(20:100, 40:120) = 30;
pos = [0; 0; -1.1; 0];
for k = 1:3
    wpt = minidroneLineFollow(R, G, B, pos);
end
ok = wpt(3) < -1;
n = report('yellow is not treated as the landing disk', ok, wpt);
end

function n = checkTrackView()
clear minidroneLineFollow
t = track([0 0; 2 0], [2.2 0 0.1]);
[R, G, B] = renderTrackCamera(0.5, 0, -1.1, 0, t);
[~, view] = minidroneLineFollow(R, G, B, [0.5; 0; -1.1; 0]);
redPix = view(:, :, 1) == 255 & view(:, :, 2) == 40;
ok = isequal(size(view), [120 160 3]) && isa(view, 'uint8') && nnz(redPix) > 200;
n = report('tracking view shows the red line', ok, [0 0 0 0]);
end

function wpt = run(pos, t, nSteps)
% Leave takeoff (climb is already done at pos) and settle the state.
for k = 1:nSteps
    [R, G, B] = renderTrackCamera(pos(1), pos(2), pos(3), pos(4), t);
    wpt = minidroneLineFollow(R, G, B, pos);
end
end

function [wx, wy] = toWorld(wpt)
wx = wpt(1) * cos(wpt(4)) - wpt(2) * sin(wpt(4));
wy = wpt(1) * sin(wpt(4)) + wpt(2) * cos(wpt(4));
end

function t = track(pts, circle)
t = struct('Name', 'unit', 'Lines', [pts(1:end-1, :), pts(2:end, :)], 'Circle', circle);
end

function n = report(name, ok, wpt)
if ok
    fprintf('  PASS  %s  wpt=[%.3f %.3f %.3f %.3f]\n', name, wpt(1), wpt(2), wpt(3), wpt(4));
    n = 0;
else
    fprintf('  FAIL  %s  wpt=[%.3f %.3f %.3f %.3f]\n', name, wpt(1), wpt(2), wpt(3), wpt(4));
    n = 1;
end
end
