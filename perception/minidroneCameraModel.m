function cam = minidroneCameraModel()
%MINIDRONECAMERAMODEL Downward camera geometry shared by guidance and tests.
%#codegen
% Fitted to logged Unreal frames (work/fitCamera.m, red-mask IoU 0.60):
% image top = vehicle forward, image right = vehicle right, effective
% horizontal FOV ~44 deg, optical axis under the vehicle centre, and the
% frame reaching guidance lags the pose by 2 guidance steps (0.4 s).
cam.rows = 120;
cam.cols = 160;
cam.tanHalfHfov = 0.40;
cam.rowFwd = 1;
cam.colRight = 1;
cam.camRight = 0;
cam.heightOffset = 0.026;
cam.lagSteps = 2;
end
