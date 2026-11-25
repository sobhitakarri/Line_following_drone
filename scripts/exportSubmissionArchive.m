function outFile = exportSubmissionArchive(teamName)
%EXPORTSUBMISSIONARCHIVE Create Round-1 Whole Project .mlproj archive.
% exportSubmissionArchive('DroneSquad')
%   -> work/DroneSquad_MDCIndia_2026.mlproj
%
% Prerequisites:
%   1) addCompetitionFilesToProject
%   2) PerceptionAndGuidance contains redLineGuidance
%   3) Offline / live checks pass

if nargin < 1 || isempty(teamName)
    error('Provide your registered team name, e.g. exportSubmissionArchive(''MyTeam'')');
end

root = fileparts(fileparts(mfilename('fullpath')));
prj = fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj');
if isempty(matlab.project.rootProject)
    openProject(prj);
end
proj = currentProject;

% Ensure custom files are registered before export.
addCompetitionFilesToProject();

safeTeam = regexprep(strtrim(teamName), '[^\w\-]+', '');
if isempty(safeTeam)
    error('Team name produced an empty filename.');
end

outDir = fullfile(root, 'work');
if ~isfolder(outDir)
    mkdir(outDir);
end
outFile = fullfile(outDir, sprintf('%s_MDCIndia_2026.mlproj', safeTeam));
if isfile(outFile)
    delete(outFile);
end

% Drop known missing stock entries so Whole Project export can succeed.
missingStock = { ...
    fullfile(root, 'support', 'ipSubsystemCallback.m')
    };
for i = 1:numel(missingStock)
    if ~isfile(missingStock{i})
        try
            removeFile(proj, missingStock{i});
            fprintf('Removed missing project entry: %s\n', missingStock{i});
        catch
        end
    end
end

% R2026b API: export(proj, archiveName)
try
    export(proj, outFile);
catch err
    fprintf('Retry export with IgnoreMissingFilesError: %s\n', err.message);
    export(proj, outFile, 'IgnoreMissingFilesError', true);
end

if ~isfile(outFile)
    error(['Could not create %s. In MATLAB Project tab use Share -> Archive ' ...
        'and save as %s_MDCIndia_2026.mlproj'], outFile, safeTeam);
end

fprintf('Submission archive ready:\n  %s\n', outFile);
info = dir(outFile);
fprintf('Size: %.1f MB\n', info.bytes / 1e6);
fprintf('Submit via competition page (team captain registered email).\n');
end
