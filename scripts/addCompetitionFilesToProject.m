function addCompetitionFilesToProject()
%ADDCOMPETITIONFILESTOPROJECT Register Round-1 custom files in the MATLAB Project.
% Required so Share -> Archive -> Whole Project includes guidance sources.

root = fileparts(fileparts(mfilename('fullpath')));
prj = fullfile(root, 'PX4MultiCopterPerceptionAndControl.prj');
if isempty(matlab.project.rootProject)
    openProject(prj);
end
proj = currentProject;

files = { ...
    fullfile(root, 'perception', 'minidroneLineFollow.m')
    fullfile(root, 'perception', 'minidroneCameraModel.m')
    fullfile(root, 'perception', 'renderTrackCamera.m')
    fullfile(root, 'scripts', 'analyzeLiveCamera.m')
    fullfile(root, 'perception', 'competitionTracks.m')
    fullfile(root, 'perception', 'simulateTrackOffline.m')
    fullfile(root, 'perception', 'simulateMultiTrackOffline.m')
    fullfile(root, 'perception', 'testMinidroneLineFollow.m')
    fullfile(root, 'scripts', 'installRedLineGuidance.m')
    fullfile(root, 'scripts', 'writeCompetitionTrack.m')
    fullfile(root, 'scripts', 'checkPerceptionCodegen.m')
    fullfile(root, 'scripts', 'addCompetitionFilesToProject.m')
    fullfile(root, 'scripts', 'exportSubmissionArchive.m')
    fullfile(root, 'scripts', 'runRound1GapChecks.m')
    fullfile(root, 'scripts', 'runLiveFlightCheck.m')
    fullfile(root, 'scripts', 'runShortFlightCheck.m')
    fullfile(root, 'support', 'ipSubsystemCallback.m')
    };

for i = 1:numel(files)
    f = files{i};
    if ~isfile(f)
        fprintf('SKIP missing: %s\n', f);
        continue
    end
    try
        addFile(proj, f);
        fprintf('ADDED: %s\n', f);
    catch err
        if contains(err.message, 'already', 'IgnoreCase', true) ...
                || contains(err.message, 'exists', 'IgnoreCase', true)
            fprintf('OK already in project: %s\n', f);
        else
            rethrow(err);
        end
    end

    % Keep custom offline test/runner scripts unlabelled so the Project
    % Test Runner (runProjectTests) stays at defaults and does not error.
    [~, name] = fileparts(f);
    if startsWith(name, 'test') || strcmp(name, 'simulateMultiTrackOffline') ...
            || strcmp(name, 'runRound1GapChecks')
        try
            pf = findFile(proj, f);
            removeLabel(pf, 'Classification', 'Test');
        catch
        end
    end
end

fprintf('Project file registration complete.\n');
fprintf('Next: exportSubmissionArchive(''YourTeamName'')\n');
end
