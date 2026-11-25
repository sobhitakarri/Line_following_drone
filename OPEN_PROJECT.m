% OPEN_PROJECT Open the competition copy of the PX4 minidrone project.
% Run this from MATLAB after the files live under D:\PEOJECTS\DRone_compitetion.

root = fileparts(mfilename('fullpath'));
prj = fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj');
if ~isfile(prj)
    error('Project file not found: %s', prj);
end

if ~isempty(matlab.project.rootProject)
    close(matlab.project.rootProject);
end

openProject(prj);
open_system('PX4MultiCopterPerceptionAndControl');
fprintf('Opened competition project from:\n%s\n', root);
