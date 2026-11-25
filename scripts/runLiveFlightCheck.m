function result = runLiveFlightCheck(stopTime)
%RUNLIVEFLIGHTCHECK Interactive-capable Unreal top-model smoke/land check.
% runLiveFlightCheck()      -> StopTime 25 s
% runLiveFlightCheck(40)    -> custom stop time
%
% Saves work/liveFlightCheck.mat and a late camera frame PNG.
% Does not permanently modify the top model (logging blocks removed after).

if nargin < 1 || isempty(stopTime)
    stopTime = 25;
end

root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
if isempty(matlab.project.rootProject)
    openProject(fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj'));
end

% Ensure default competition track is present for Unreal scene.
if ~isfile(fullfile(root, 'initData.mat'))
    writeCompetitionTrack('L_default');
end

mdl = 'PX4MultiCopterPerceptionAndControl';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

% Temporary To Workspace loggers
% Log exactly what PerceptionGuidance sees: camera (in 1), pose (in 2),
% and what it commands (out 1).
pg = [mdl '/PerceptionGuidance'];
pgPorts = get_param(pg, 'PortHandles');
camPort = get_param(get_param(pgPorts.Inport(1), 'Line'), 'SrcPortHandle');
posPort = get_param(get_param(pgPorts.Inport(2), 'Line'), 'SrcPortHandle');
twNames = {'liveCamLog', 'livePosLog', 'liveCmdLog'};

addTw(mdl, twNames{1}, camPort, [2600 960 2660 990]);
addTw(mdl, twNames{2}, posPort, [1040 1120 1100 1150]);
addTw(mdl, twNames{3}, pgPorts.Outport(1), [1320 900 1380 930]);

cleanup = onCleanup(@() cleanupModel(mdl, twNames)); %#ok<NASGU>

fprintf('Starting live top-model sim for %.1f s (Unreal may open)...\n', stopTime);
simOut = sim(mdl, 'StopTime', num2str(stopTime), 'ReturnWorkspaceOutputs', 'on', ...
    'CaptureErrors', 'on');

if ~isempty(simOut.ErrorMessage)
    disp(simOut.ErrorMessage);
    error('Live flight failed: %s', simOut.ErrorMessage);
end

outDir = fullfile(root, 'work');
if ~isfolder(outDir)
    mkdir(outDir);
end

fprintf('Sim outputs: %s\n', strjoin(simOut.who, ', '));
cam = getLog(simOut, 'liveCamLog');
pos = getLog(simOut, 'livePosLog');
cmd = getLog(simOut, 'liveCmdLog');

frame = lastFrame(cam);
imwrite(uint8(frame), fullfile(outDir, 'liveFlightCheckFrame.png'));
save(fullfile(outDir, 'liveFlightCheck.mat'), 'pos', 'cmd', 'cam', 'frame', 'stopTime');

posData = asMatrix(pos);
cmdData = asMatrix(cmd);
result = struct();
result.StopTime = stopTime;
result.HasPos = ~isempty(posData);
result.HasCmd = ~isempty(cmdData);
result.FrameSize = size(frame);

if ~isempty(posData)
    % Expect NED Z negative while flying; near end Z closer to 0 if landed.
    z = posData(:, min(3, size(posData, 2)));
    result.MinZ = min(z);
    result.EndZ = z(end);
    result.EndXY = posData(end, 1:min(2, size(posData, 2)));
    fprintf('Pos end XY=[%.2f %.2f] Z=%.3f (minZ=%.3f)\n', ...
        result.EndXY(1), result.EndXY(2), result.EndZ, result.MinZ);
    init = load(fullfile(root, 'initData.mat'));
    if isfield(init, 'Circle') && numel(result.EndXY) == 2
        % Scene frame = plant frame + [0.5, 1.0] (Visualization Subsystem).
        target = [init.Circle.X, init.Circle.Y] - [0.5, 1.0];
        result.LandErr = hypot(result.EndXY(1) - target(1), result.EndXY(2) - target(2));
        result.Landed = result.EndZ > -0.05;
        fprintf('Circle centre (plant) [%.2f %.2f]: landing error %.3f m, landed=%d\n', ...
            target(1), target(2), result.LandErr, result.Landed);
    end
end
if ~isempty(cmdData)
    result.LastCmd = cmdData(end, :);
    fprintf('Last cmd ~ %s\n', mat2str(result.LastCmd, 3));
end

fprintf('Wrote %s\n', fullfile(outDir, 'liveFlightCheckFrame.png'));
fprintf('PASS: live top-model sim completed without error.\n');
fprintf('Manually confirm Video Viewer / 3D: followed red line and landed on circle.\n');
end

function addTw(mdl, name, outPort, pos)
path = [mdl '/' name];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
    add_block('simulink/Sinks/To Workspace', path, 'Position', pos);
end
set_param(path, 'VariableName', name, 'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf', ...
    'SampleTime', '0.2');
try
    add_line(mdl, outPort, get_param(path, 'PortHandles').Inport(1), 'autorouting', 'on');
catch
end
end

function cleanupModel(mdl, twNames)
if ~bdIsLoaded(mdl)
    return
end
for i = 1:numel(twNames)
    path = [mdl '/' twNames{i}];
    if ~isempty(find_system(mdl, 'SearchDepth', 1, 'Name', twNames{i}))
        try
            ph = get_param(path, 'PortHandles');
            lh = get_param(ph.Inport, 'Line');
            if lh ~= -1
                delete_line(lh);
            end
        catch
        end
        delete_block(path);
    end
end
% Discard temporary diagram edits.
close_system(mdl, 0);
end

function v = getLog(simOut, name)
v = [];
if any(strcmp(simOut.who, name))
    v = simOut.get(name);
end
end

function frame = lastFrame(cam)
if isempty(cam)
    frame = zeros(120, 160, 3, 'uint8');
    return
end
if isa(cam, 'timeseries')
    frame = squeeze(cam.Data(:, :, :, end));
elseif isstruct(cam) && isfield(cam, 'signals')
    frame = cam.signals.values(:, :, :, end);
else
    frame = cam(:, :, :, end);
end
end

function M = asMatrix(logData)
M = [];
if isempty(logData)
    return
end
if isstruct(logData)
    fn = fieldnames(logData);
    for i = 1:numel(fn)
        M = [M, double(logData.(fn{i}).Data(:))]; %#ok<AGROW>
    end
elseif isa(logData, 'timeseries')
    M = squeeze(logData.Data);
    if ndims(M) > 2 %#ok<ISMAT>
        M = reshape(M, [], size(M, ndims(M))).';
    elseif size(M, 1) < size(M, 2) && size(M, 1) <= 8
        M = M.';
    end
elseif isnumeric(logData)
    M = logData;
end
end
