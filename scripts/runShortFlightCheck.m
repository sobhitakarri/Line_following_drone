function runShortFlightCheck()
%RUNSHORTFLIGHTCHECK Short top-model sim; saves work/flightCheckFrame.png.
% Uses To Workspace blocks temporarily; does not permanently save the model.

root = fileparts(fileparts(mfilename('fullpath')));
cd(root);
if isempty(matlab.project.rootProject)
    openProject(fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj'));
end

mdl = 'PX4MultiCopterPerceptionAndControl';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end
cleanup = onCleanup(@() closeIfLoaded(mdl)); %#ok<NASGU>

% Temporary logging blocks. Parent is the top model diagram.
camSrc = [mdl '/Visualization Subsystem'];
posSrc = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'BusSelector');
cmdSrc = [mdl '/PerceptionGuidance'];

addTw(mdl, 'camLog', get_param(camSrc, 'PortHandles').Outport(1), [2600 960 2660 990]);
addTw(mdl, 'posLog', get_param(posSrc{1}, 'PortHandles').Outport(1), [1040 1120 1100 1150]);
addTw(mdl, 'cmdLog', get_param(cmdSrc, 'PortHandles').Outport(1), [1320 900 1380 930]);

simOut = sim(mdl, 'StopTime', '6', 'ReturnWorkspaceOutputs', 'on', ...
    'CaptureErrors', 'on');

if ~isempty(simOut.ErrorMessage)
    disp(simOut.ErrorMessage);
    error('Short flight failed: %s', simOut.ErrorMessage);
end

outDir = fullfile(root, 'work');
if ~isfolder(outDir)
    mkdir(outDir);
end

cam = simOut.camLog;
if isa(cam, 'timeseries')
    frame = squeeze(cam.Data(:, :, :, end));
elseif isstruct(cam) && isfield(cam, 'signals')
    frame = cam.signals.values(:, :, :, end);
else
    frame = cam(:, :, :, end);
end
imwrite(uint8(frame), fullfile(outDir, 'flightCheckFrame.png'));

cmd = simOut.cmdLog;
pos = simOut.posLog;
save(fullfile(outDir, 'flightCheck.mat'), 'pos', 'cmd', 'frame');

fprintf('Frame size %s\n', mat2str(size(frame)));
if isa(cmd, 'timeseries')
    last = cmd.Data(end, :);
    fprintf('Last cmd ~ [%.3f %.3f %.3f %.3f]\n', last(1), last(2), last(3), last(4));
end
fprintf('Wrote %s\n', fullfile(outDir, 'flightCheckFrame.png'));
end

function addTw(mdl, name, outPort, pos)
path = [mdl '/' name];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', name))
    add_block('simulink/Sinks/To Workspace', path, 'Position', pos);
end
set_param(path, 'VariableName', name, 'SaveFormat', 'Timeseries', 'MaxDataPoints', 'inf');
lineH = get_param(outPort, 'Line');
if lineH == -1
    error('No line on port for %s', name);
end
% Branch from existing line by connecting destination to the logged signal.
try
    add_line(mdl, outPort, get_param(path, 'PortHandles').Inport(1), 'autorouting', 'on');
catch
    % Already connected from a previous partial run.
end
end

function closeIfLoaded(mdl)
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
end
