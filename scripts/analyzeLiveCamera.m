function analyzeLiveCamera()
%ANALYZELIVECAMERA Replay work/liveFlightCheck.mat through the guidance.
% Prints the flight timeline (pose, command, guidance mode), checks the
% synthetic camera model against real Unreal frames (red-mask IoU), and
% writes a montage of tracking views to work/liveTrackViews.png.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));
S = load(fullfile(root, 'work', 'liveFlightCheck.mat'));
init = load(fullfile(root, 'initData.mat'));
disp(init.Circle);
disp(init.Lines);

P = [S.pos.X.Data(:), S.pos.Y.Data(:), S.pos.Z.Data(:), S.pos.yaw.Data(:)];
C = [S.cmd.X.Data(:), S.cmd.Y.Data(:), S.cmd.Z.Data(:), S.cmd.Yaw.Data(:)];
t = S.pos.X.Time(:);
frames = S.cam.Data;
n = min([size(P, 1), size(C, 1), size(frames, 4)]);

track = sceneTrack(init);
clear minidroneLineFollow
views = cell(1, n);
iou = nan(n, 1);
for k = 1:n
    R = frames(:, :, 1, k);
    G = frames(:, :, 2, k);
    B = frames(:, :, 3, k);
    [traj, views{k}] = minidroneLineFollow(R, G, B, P(k, :)');
    mode = modeFromView(views{k});
    if P(k, 3) < -0.3
        [Rs, Gs, Bs] = renderTrackCamera(P(k, 1), P(k, 2), P(k, 3), P(k, 4), track);
        a = redMaskOf(R, G, B);
        b = redMaskOf(Rs, Gs, Bs);
        if any(a(:) | b(:))
            iou(k) = nnz(a & b) / nnz(a | b);
        end
    end
    if mod(k, 5) == 1 || k == n
        fprintf('t=%5.1f pos=(%5.2f,%5.2f,%5.2f) yaw=%6.1f | cmd=(%5.2f,%5.2f,%5.2f) yaw=%6.1f | mode=%d iou=%.2f | traj=%s\n', ...
            t(k), P(k, 1), P(k, 2), P(k, 3), P(k, 4) * 180 / pi, C(k, 1), C(k, 2), C(k, 3), ...
            C(k, 4) * 180 / pi, mode, iou(k), mat2str(traj', 3));
    end
end
fprintf('Median red-mask IoU (real vs synthetic camera): %.2f\n', median(iou, 'omitnan'));

sel = unique(round(linspace(1, n, 24)));
tiles = cell(1, numel(sel));
for i = 1:numel(sel)
    tiles{i} = [views{sel(i)}, frames(:, :, :, sel(i))];
end
fig = figure('Visible', 'off');
montage(tiles, 'Size', [6 4]);
exportgraphics(gca, fullfile(root, 'work', 'liveTrackViews.png'));
close(fig);
fprintf('Wrote work/liveTrackViews.png (left = tracking view, right = raw camera)\n');
end

function track = sceneTrack(init)
lines = init.Lines;
if isstruct(lines)
    lines = struct2mat(lines);
end
circ = init.Circle;
if isstruct(circ)
    circ = struct2mat(circ);
end
off = [0.5, 1.0];
track.Name = 'live';
track.Lines = lines(:, 1:4) - [off, off];
track.Circle = [circ(1, 1:2) - off, circ(1, end) / 2];
end

function m = struct2mat(s)
c = struct2cell(s(:));
m = cell2mat(cellfun(@(v) double(v(:))', c(:)', 'UniformOutput', false));
m = reshape(m, [], numel(s)).';
end

function m = redMaskOf(R, G, B)
r = int16(R); g = int16(G); b = int16(B);
m = (R >= 80) & (r - g >= 50) & (r - b >= 50);
end

function mode = modeFromView(v)
colors = [255 255 255; 0 255 0; 255 140 0; 255 230 0; 255 0 255; 60 120 255];
px = double(squeeze(v(3, 3, :)))';
[~, mode] = min(sum(abs(colors - px), 2));
end
