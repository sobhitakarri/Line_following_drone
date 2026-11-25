function results = simulateMultiTrackOffline()
%SIMULATEMULTITRACKOFFLINE Run the offline closed loop on every competitionTracks track.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));

tracks = competitionTracks();
results = [];
for i = 1:numel(tracks)
    r = simulateTrackOffline(tracks(i));
    results = [results; r]; %#ok<AGROW>
end

nPass = sum([results.Passed]);
fprintf('\nMulti-track offline: %d / %d passed\n', nPass, numel(results));
save(fullfile(root, 'work', 'multiTrackOffline.mat'), 'results');
if nPass < numel(results)
    bad = results(~[results.Passed]);
    msg = strjoin(arrayfun(@(r) sprintf('%s: %s', r.Name, r.Reason), bad, 'UniformOutput', false), newline);
    error('simulateMultiTrackOffline failed:\n%s', msg);
end
end
