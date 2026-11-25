function writeCompetitionTrack(trackName)
%WRITECOMPETITIONTRACK Update Lines/Circle in initData.mat for Unreal scene.
% Preserves other fields (posNED, Color, ...) required by InitVars /
% competitionScene. Safe to call repeatedly.
%
% writeCompetitionTrack()            -> L_default
% writeCompetitionTrack('zigzag')    -> named track from competitionTracks
%
% After writing, reopen/run the top model so Unreal picks up the new track.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));

if nargin < 1 || isempty(trackName)
    trackName = 'L_default';
end

tracks = competitionTracks();
names = {tracks.Name};
idx = find(strcmpi(names, trackName), 1);
if isempty(idx)
    error('Unknown track "%s". Available: %s', trackName, strjoin(names, ', '));
end
t = tracks(idx);

Lines = struct('StartX', {}, 'StartY', {}, 'EndX', {}, 'EndY', {});
for i = 1:size(t.SceneLines, 1)
    Lines(i).StartX = t.SceneLines(i, 1); %#ok<AGROW>
    Lines(i).StartY = t.SceneLines(i, 2);
    Lines(i).EndX = t.SceneLines(i, 3);
    Lines(i).EndY = t.SceneLines(i, 4);
end

Circle = struct( ...
    'X', t.SceneCircle(1), ...
    'Y', t.SceneCircle(2), ...
    'Z', t.SceneCircle(3), ...
    'Diameter', t.SceneCircle(4));

outFile = fullfile(root, 'initData.mat');
data = struct();
if isfile(outFile)
    data = load(outFile);
end

data.Lines = Lines;
data.Circle = Circle;
if ~isfield(data, 'Color') || isempty(data.Color)
    data.Color = [1 0 0];
end
if ~isfield(data, 'posNED') || isempty(data.posNED)
    % Match InitVars fallback when no prior pose is stored.
    data.posNED = [0, 0, 0];
end

save(outFile, '-struct', 'data');
fprintf('Wrote %s for track "%s" (%d segments, circle diam %.2f)\n', ...
    outFile, t.Name, numel(Lines), Circle.Diameter);
fprintf('Preserved fields: %s\n', strjoin(fieldnames(data), ', '));
fprintf('Re-open / Run PX4MultiCopterPerceptionAndControl to load the track.\n');
fprintf('Optional GUI: openTrackBuilder\n');
end
