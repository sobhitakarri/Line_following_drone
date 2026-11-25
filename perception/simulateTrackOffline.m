function result = simulateTrackOffline(trackOrName, opts)
%SIMULATETRACKOFFLINE Closed-loop check of minidroneLineFollow without Unreal.
% simulateTrackOffline()                 -> stock L track
% simulateTrackOffline('zigzag')         -> named track from competitionTracks
% simulateTrackOffline(track, struct('Plot', true))
%
% Plant: 5 Hz guidance, localToNED rotation (yaw0 = 0), first-order position
% and yaw response with speed limits, one-frame camera delay.
% PASS = touched down within 0.08 m of the circle centre, never more than
% 0.15 m off the track centreline while cruising.

if nargin < 1 || isempty(trackOrName)
    trackOrName = 'L_default';
end
if nargin < 2
    opts = struct();
end
if ~isfield(opts, 'Plot'), opts.Plot = false; end
if ~isfield(opts, 'Quiet'), opts.Quiet = false; end
if ~isfield(opts, 'SaveViews'), opts.SaveViews = false; end

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));
outDir = fullfile(root, 'work');
if ~isfolder(outDir)
    mkdir(outDir);
end

track = resolveTrack(trackOrName);
clear minidroneLineFollow

dt = 0.2;
maxSteps = 1500;
state = [0; 0; 0.002; 0];
hist = zeros(maxSteps, 9);
crossMax = 0;
yawErrs = zeros(maxSteps, 1);
nYaw = 0;
grounded = 0;
n = 0;
lastView = zeros(120, 160, 3, 'uint8');

cam = minidroneCameraModel();
past = repmat(state, 1, cam.lagSteps + 1);
yawCmdPrev = 0;
yawStepPrev = 0;
reversals = 0;
for k = 1:maxSteps
    past = [past(:, 2:end), state];
    [R, G, B] = renderTrackCamera(past(1, 1), past(2, 1), past(3, 1), past(4, 1), track);
    [traj, lastView] = minidroneLineFollow(R, G, B, state);
    yawStep = wrapPi(traj(4) - yawCmdPrev);
    if abs(yawStep) > 0.05
        if yawStepPrev ~= 0 && sign(yawStep) ~= sign(yawStepPrev)
            reversals = reversals + 1;
        end
        yawStepPrev = yawStep;
    end
    yawCmdPrev = traj(4);
    if opts.SaveViews
        imwrite(lastView, fullfile(outDir, sprintf('view_%03d.png', k)));
    end

    h = traj(4);
    wx = traj(1) * cos(h) - traj(2) * sin(h);
    wy = traj(1) * sin(h) + traj(2) * cos(h);
    wz = traj(3);

    dxy = 0.6 * [wx - state(1); wy - state(2)];
    v = norm(dxy);
    if v > 0.6 * dt
        dxy = dxy * (0.6 * dt / v);
    end
    state(1:2) = state(1:2) + dxy;
    state(3) = state(3) + max(-0.08, min(0.08, 0.6 * (wz - state(3))));
    state(3) = min(state(3), 0.002);
    state(4) = state(4) + max(-0.35, min(0.35, 0.7 * wrapPi(h - state(4))));

    n = n + 1;
    hist(n, :) = [state', wx, wy, wz, h, k * dt];

    [dLine, segDir] = trackDistance(state(1), state(2), track.Lines);
    nearCircle = hypot(state(1) - track.Circle(1), state(2) - track.Circle(2)) < 0.3;
    if state(3) < -0.9 && ~nearCircle
        crossMax = max(crossMax, dLine);
        nYaw = nYaw + 1;
        yawErrs(nYaw) = abs(wrapPi(state(4) - segDir));
    end

    if state(3) > -0.03
        grounded = grounded + 1;
    else
        grounded = 0;
    end
    if k > 20 && grounded >= 3
        break
    end
end

hist = hist(1:n, :);
landErr = hypot(state(1) - track.Circle(1), state(2) - track.Circle(2));
landed = grounded >= 3;
medYaw = 0;
if nYaw > 0
    medYaw = median(yawErrs(1:nYaw)) * 180 / pi;
end
nCorners = size(track.Lines, 1) - 1;
maxReversals = 2 * nCorners + 2;
passed = landed && landErr <= 0.08 && crossMax <= 0.15 && reversals <= maxReversals;
reason = 'ok';
if ~landed
    reason = 'never landed';
elseif landErr > 0.08
    reason = sprintf('landed %.2f m from circle centre', landErr);
elseif crossMax > 0.15
    reason = sprintf('left the line by %.2f m', crossMax);
elseif reversals > maxReversals
    reason = sprintf('heading wobbles (%d reversals)', reversals);
end

safeName = regexprep(track.Name, '[^\w]+', '_');
pngPath = fullfile(outDir, ['offlineTrack_' safeName '.png']);
imwrite(lastView, pngPath);
save(fullfile(outDir, ['offlineTrack_' safeName '.mat']), 'hist', 'track');
if opts.Plot
    plotRun(hist, track);
end

result = struct( ...
    'Name', track.Name, ...
    'Passed', passed, ...
    'Landed', landed, ...
    'LandErr', landErr, ...
    'MaxCrossTrack', crossMax, ...
    'MedianYawErrDeg', medYaw, ...
    'YawReversals', reversals, ...
    'FlightTime', n * dt, ...
    'Reason', reason, ...
    'PngPath', pngPath);

if ~opts.Quiet
    fprintf('[%-15s] %s | land err %.3f m | max off-line %.3f m | yaw err %4.1f deg | yaw reversals %2d | %5.1f s | %s\n', ...
        track.Name, ternary(passed, 'PASS', 'FAIL'), landErr, crossMax, medYaw, reversals, n * dt, reason);
end
end

function [dMin, segDir] = trackDistance(x, y, lines)
dMin = inf;
segDir = 0;
for i = 1:size(lines, 1)
    x1 = lines(i, 1); y1 = lines(i, 2);
    dx = lines(i, 3) - x1; dy = lines(i, 4) - y1;
    L2 = dx * dx + dy * dy;
    t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / L2));
    d = hypot(x - (x1 + t * dx), y - (y1 + t * dy));
    if d < dMin
        dMin = d;
        segDir = atan2(dy, dx);
    end
end
end

function plotRun(hist, track)
figure('Name', ['Offline run: ' track.Name]);
hold on; axis equal; grid on;
for i = 1:size(track.Lines, 1)
    plot(track.Lines(i, [2 4]), track.Lines(i, [1 3]), 'r-', 'LineWidth', 6);
end
th = linspace(0, 2 * pi, 60);
fill(track.Circle(2) + track.Circle(3) * sin(th), track.Circle(1) + track.Circle(3) * cos(th), 'r');
plot(hist(:, 2), hist(:, 1), 'b-', 'LineWidth', 1.5);
plot(hist(end, 2), hist(end, 1), 'ko', 'MarkerFaceColor', 'y');
xlabel('Y east (m)'); ylabel('X north (m)');
title(sprintf('%s (blue = flown path)', track.Name), 'Interpreter', 'none');
end

function track = resolveTrack(trackOrName)
if isstruct(trackOrName)
    track = trackOrName;
    if ~isfield(track, 'Name')
        track.Name = 'custom';
    end
    return
end
allTracks = competitionTracks();
names = {allTracks.Name};
idx = find(strcmpi(names, trackOrName), 1);
if isempty(idx)
    error('Unknown track "%s". Available: %s', trackOrName, strjoin(names, ', '));
end
track = allTracks(idx);
end

function a = wrapPi(a)
a = mod(a + pi, 2 * pi) - pi;
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end
