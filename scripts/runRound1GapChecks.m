function summary = runRound1GapChecks(opts)
%RUNROUND1GAPCHECKS Fill / verify Round-1 readiness gaps.
% runRound1GapChecks()
% runRound1GapChecks(struct('SkipLive', true))
% runRound1GapChecks(struct('SkipLive', false, 'LiveStopTime', 25))

if nargin < 1
    opts = struct();
end
if ~isfield(opts, 'SkipLive')
    opts.SkipLive = true; % Unreal batch is flaky; run live interactively later
end
if ~isfield(opts, 'LiveStopTime')
    opts.LiveStopTime = 25;
end
if ~isfield(opts, 'TeamName')
    opts.TeamName = '';
end

root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
addpath(fullfile(root, 'perception'));
addpath(fullfile(root, 'scripts'));

if isempty(matlab.project.rootProject)
    openProject(fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj'));
end

summary = struct();
summary.ProjectFiles = false;
summary.UnitTests = false;
summary.MultiTrack = false;
summary.Codegen = false;
summary.Live = false;
summary.TrackFile = false;
summary.Errors = {};

fprintf('\n==== 1) Register project files ====\n');
try
    addCompetitionFilesToProject();
    summary.ProjectFiles = true;
catch err
    summary.Errors{end+1} = ['ProjectFiles: ' err.message];
    fprintf('FAIL project files: %s\n', err.message);
end

fprintf('\n==== 2) Write default Unreal track (initData.mat) ====\n');
try
    writeCompetitionTrack('L_default');
    summary.TrackFile = isfile(fullfile(root, 'initData.mat'));
catch err
    summary.Errors{end+1} = ['TrackFile: ' err.message];
end

fprintf('\n==== 3) Unit tests ====\n');
try
    testMinidroneLineFollow();
    summary.UnitTests = true;
catch err
    summary.Errors{end+1} = ['UnitTests: ' err.message];
end

fprintf('\n==== 4) Multi-track offline ====\n');
try
    simulateMultiTrackOffline();
    summary.MultiTrack = true;
catch err
    summary.Errors{end+1} = ['MultiTrack: ' err.message];
end

fprintf('\n==== 5) Codegen ====\n');
try
    checkPerceptionCodegen();
    summary.Codegen = true;
catch err
    summary.Errors{end+1} = ['Codegen: ' err.message];
end

fprintf('\n==== 6) Live Unreal top-model ====\n');
if opts.SkipLive
    fprintf('SKIPPED (set SkipLive=false to run). Use interactively:\n');
    fprintf('  runLiveFlightCheck(25)\n');
    fprintf('  or open_system(''PX4MultiCopterPerceptionAndControl''); %% then Run\n');
else
    try
        runLiveFlightCheck(opts.LiveStopTime);
        summary.Live = true;
    catch err
        summary.Errors{end+1} = ['Live: ' err.message];
        fprintf('FAIL live: %s\n', err.message);
    end
end

if ~isempty(opts.TeamName)
    fprintf('\n==== 7) Export submission archive ====\n');
    try
        exportSubmissionArchive(opts.TeamName);
    catch err
        summary.Errors{end+1} = ['Export: ' err.message];
    end
end

outDir = fullfile(root, 'work');
if ~isfolder(outDir)
    mkdir(outDir);
end
save(fullfile(outDir, 'round1GapChecks.mat'), 'summary', 'opts');

fprintf('\n======== ROUND 1 GAP CHECK SUMMARY ========\n');
fprintf('Project files : %s\n', tf(summary.ProjectFiles));
fprintf('initData.mat  : %s\n', tf(summary.TrackFile));
fprintf('Unit tests    : %s\n', tf(summary.UnitTests));
fprintf('Multi-track   : %s\n', tf(summary.MultiTrack));
fprintf('Codegen       : %s\n', tf(summary.Codegen));
fprintf('Live Unreal   : %s\n', tf(summary.Live));
if isempty(summary.Errors)
    fprintf('No hard failures in non-skipped checks.\n');
else
    fprintf('Issues:\n');
    fprintf('  - %s\n', strjoin(summary.Errors, '\n  - '));
end
fprintf('===========================================\n');
end

function s = tf(v)
if v
    s = 'PASS';
else
    s = 'FAIL/SKIP';
end
end
