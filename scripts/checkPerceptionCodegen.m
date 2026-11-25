function checkPerceptionCodegen()
%CHECKPERCEPTIONCODEGEN Verify Round-1 codegen readiness for guidance logic.
% 1) MATLAB Coder build of minidroneLineFollow
% 2) GenCodeOnly update/build attempt on PerceptionAndGuidance (if available)

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'perception'));
outDir = fullfile(root, 'work', 'codegen_check');
if ~isfolder(outDir)
    mkdir(outDir);
end

fprintf('=== Codegen: minidroneLineFollow ===\n');
R = zeros(120, 160, 'uint8');
G = zeros(120, 160, 'uint8');
B = zeros(120, 160, 'uint8');
pos = zeros(4, 1);

cfg = coder.config('lib', 'ecoder', true);
cfg.GenCodeOnly = true;
cfg.GenerateReport = false;
cfg.LaunchReport = false;

codegenArgs = { ...
    '-config', cfg, ...
    '-d', fullfile(outDir, 'minidroneLineFollow'), ...
    'minidroneLineFollow', ...
    '-args', {R, G, B, pos}};

try
    codegen(codegenArgs{:});
    fprintf('PASS: minidroneLineFollow codegen (GenCodeOnly)\n');
catch err
    error('FAIL: minidroneLineFollow codegen: %s', err.message);
end

fprintf('=== Model resolve: PerceptionAndGuidance ===\n');
if isempty(matlab.project.rootProject)
    openProject(fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj'));
end

mdl = 'PerceptionAndGuidance';
mdlFile = fullfile(root, 'perception', [mdl '.slx']);
load_system(mdlFile);
cleanup = onCleanup(@() close_system(mdl, 0)); %#ok<NASGU>

% Round-1 submission needs algorithm codegen. Full Raspberry Pi rtwbuild is
% Round-2 deploy; skip board toolchain here and only confirm the model loads
% with redLineGuidance present.
try
    set_param(mdl, 'HardwareBoard', 'None');
catch
end

try
    set_param(mdl, 'SimulationCommand', 'update');
    fprintf('PASS: PerceptionAndGuidance model update (HardwareBoard=None)\n');
catch err
    fprintf('WARN: PerceptionAndGuidance update: %s\n', err.message);
end

rg = find_system(mdl, 'SearchDepth', 2, 'Name', 'redLineGuidance');
if isempty(rg)
    error('FAIL: redLineGuidance block missing — run installRedLineGuidance');
end
fprintf('PASS: redLineGuidance block present\n');
fprintf('NOTE: Install "MATLAB Support Package for Raspberry Pi" before Round-2 Pi deploy.\n');
fprintf('Codegen check complete. Artifacts under %s\n', outDir);
end
