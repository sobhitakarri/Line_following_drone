function installRedLineGuidance()
%INSTALLREDLINEGUIDANCE Wire red-line guidance into PerceptionAndGuidance.
%
% Replaces the template blue-square detector inside
% "Image processing and Guidance" with a MATLAB Function block that calls
% perception/minidroneLineFollow.m, plus a "Tracking View" Video Viewer that
% shows what the guidance sees (red = detected line, yellow = circle core,
% cyan ring = crossing search, green = chosen branch, top bar = mode).
% Camera variant handling (simulation vs Raspberry Pi) is left as shipped.
% Safe to run more than once.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));
addpath(fullfile(root, 'scripts'));

projFile = fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj');
if isempty(matlab.project.rootProject)
    openProject(projFile);
end

model = 'PerceptionAndGuidance';
modelFile = fullfile(root, 'perception', 'PerceptionAndGuidance.slx');
if ~bdIsLoaded(model)
    load_system(modelFile);
end

parent = [model '/Image processing and Guidance'];
keep = {'Image processing and Guidance', 'CurrentPosition', 'R', 'Enable', ...
    'get camera data', 'Trajectory', 'redLineGuidance', 'Tracking View'};

blk = [parent '/redLineGuidance'];
if isempty(find_system(parent, 'SearchDepth', 1, 'Name', 'redLineGuidance'))
    blocks = find_system(parent, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Block');
    for k = 1:numel(blocks)
        name = get_param(blocks(k), 'Name');
        if ~any(strcmp(name, keep))
            fprintf('Removing %s\n', name);
            delete_block(blocks(k));
        end
    end

    trajIn = get_param([parent '/Trajectory'], 'PortHandles');
    trajLine = get_param(trajIn.Inport, 'Line');
    if trajLine ~= -1
        delete_line(trajLine);
    end

    add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
        'Position', [620, 1220, 980, 1460], ...
        'Orientation', 'right');
end

chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = sprintf([ ...
    'function [trajectory, trackView] = redLineGuidance(R, G, B, pos)\n' ...
    '%%#codegen\n' ...
    '[trajectory, trackView] = minidroneLineFollow(R, G, B, pos);\n' ...
    'end\n']);

viewer = [parent '/Tracking View'];
if isempty(find_system(parent, 'SearchDepth', 1, 'Name', 'Tracking View'))
    add_block('visionsinks/Video Viewer', viewer, ...
        'Position', [1100, 1400, 1180, 1460]);
end

connect(parent, 'get camera data/1', 'redLineGuidance/1');
connect(parent, 'get camera data/2', 'redLineGuidance/2');
connect(parent, 'get camera data/3', 'redLineGuidance/3');
connect(parent, 'CurrentPosition/1', 'redLineGuidance/4');
connect(parent, 'redLineGuidance/1', 'Trajectory/1');
connect(parent, 'redLineGuidance/2', 'Tracking View/1');

set_param(model, 'SimulationCommand', 'update');
save_system(model, modelFile);
if bdIsLoaded('PX4MultiCopterPerceptionAndControl')
    close_system('PX4MultiCopterPerceptionAndControl', 0);
end
fprintf('Installed redLineGuidance + Tracking View in %s\n', modelFile);
end

function connect(parent, src, dst)
try
    add_line(parent, src, dst, 'autorouting', 'on');
catch err
    if ~contains(err.message, 'already has a line connection')
        rethrow(err);
    end
end
end
