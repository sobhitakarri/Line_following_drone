function tracks = competitionTracks()
%COMPETITIONTRACKS Plant-frame track library for offline Round-1 checks.
% Each track: Name, Lines [x1 y1 x2 y2; ...], Circle [x y radius],
% SceneLines / SceneCircle for writing initData.mat (Unreal / Track Builder).
% Scene frame = plant + [0.5, 1.0] (Visualization Subsystem translate_point).
% Every track starts at the spawn point (0, 0) heading +X.
%   'gap'      circle centre 0.2 m past the last segment (0.1 m bare gap, as
%              in the stock scene)
%   'attached' circle centred on the end of the last segment

d120 = [cos(2 * pi / 3), sin(2 * pi / 3)];
d145 = [cos(145 * pi / 180), sin(145 * pi / 180)];

tracks = [ ...
    stockTrack()
    makeTrack('straight_gap', [0 0; 2.4 0], 'gap')
    makeTrack('L_attached', [0 0; 1.6 0; 1.6 1.6], 'attached')
    makeTrack('left_turn', [0 0; 1.2 0; 1.2 -0.5], 'gap')
    makeTrack('zigzag', [0 0; 1.0 0; 1.0 0.8; 2.0 0.8; 2.0 1.8], 'gap')
    makeTrack('angled_45', [0 0; 1.2 0; 2.0 0.8; 2.0 1.8], 'gap')
    makeTrack('sharp_120', [0 0; 1.8 0; [1.8 0] + 1.2 * d120], 'gap')
    makeTrack('sharp_145', [0 0; 1.8 0; [1.8 0] + 1.2 * d145], 'gap')
    makeTrack('U_turn', [0 0; 2.0 0; 2.0 1.2; 0.4 1.2], 'attached')
    makeTrack('seven_sections', [0 0; 0.8 0; 0.8 0.6; 1.6 0.6; 1.6 1.4; 2.4 1.4; 2.4 2.2; 1.6 2.2], 'gap')
    ];
end

function t = stockTrack()
% competitionScene default L, shifted into the plant frame.
lines = [0.0, 0.0, 2.0, 0.0; 1.95, 0.0, 2.0, 2.0];
t = finishTrack('L_default', lines, [1.95, 2.2, 0.1]);
end

function t = makeTrack(name, pts, circleMode)
lines = [pts(1:end-1, :), pts(2:end, :)];
d = pts(end, :) - pts(end-1, :);
d = d / norm(d);
if strcmp(circleMode, 'gap')
    centre = pts(end, :) + 0.2 * d;
else
    centre = pts(end, :);
end
t = finishTrack(name, lines, [centre, 0.1]);
end

function t = finishTrack(name, lines, circle)
off = [0.5, 1.0];
sceneLines = lines;
sceneLines(:, [1 3]) = lines(:, [1 3]) + off(1);
sceneLines(:, [2 4]) = lines(:, [2 4]) + off(2);
t = struct( ...
    'Name', name, ...
    'Lines', lines, ...
    'Circle', circle, ...
    'SceneLines', sceneLines, ...
    'SceneCircle', [circle(1) + off(1), circle(2) + off(2), 0.01, circle(3) * 2]);
end
