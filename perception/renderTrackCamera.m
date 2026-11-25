function [R, G, B] = renderTrackCamera(x, y, z, yaw, track)
%RENDERTRACKCAMERA Synthetic downward camera frame over a competitionTracks track.
% Uses minidroneCameraModel and draws the scene the way competitionScene does:
% box segments 0.1 m wide, joint disks at interior joints, landing circle,
% yellow/blue start squares, textured floor.

cam = minidroneCameraModel();
height = max(0.15, -z + cam.heightOffset);
mpp = height * cam.tanHalfHfov / (cam.cols / 2);
[rr, cc] = ndgrid(1:cam.rows, 1:cam.cols);
fwd = cam.rowFwd * ((cam.rows + 1) / 2 - rr) * mpp;
right = cam.colRight * (cc - (cam.cols + 1) / 2) * mpp + cam.camRight;
wx = x + fwd * cos(yaw) - right * sin(yaw);
wy = y + fwd * sin(yaw) + right * cos(yaw);

half = 0.05;
red = false(size(wx));
lines = track.Lines;
for i = 1:size(lines, 1)
    x1 = lines(i, 1); y1 = lines(i, 2);
    dx = lines(i, 3) - x1; dy = lines(i, 4) - y1;
    L = hypot(dx, dy);
    ux = dx / L; uy = dy / L;
    along = (wx - x1) * ux + (wy - y1) * uy;
    perp = abs(-(wx - x1) * uy + (wy - y1) * ux);
    red = red | (along >= 0 & along <= L & perp <= half);
end
for i = 2:size(lines, 1)
    red = red | hypot(wx - lines(i, 1), wy - lines(i, 2)) <= half;
end
circ = track.Circle;
red = red | hypot(wx - circ(1), wy - circ(2)) <= circ(3);

yel = abs(wx - 0.0) <= 0.1 & abs(wy - 1.0) <= 0.1;
blu = abs(wx - 0.0) <= 0.1 & abs(wy - 1.5) <= 0.1;

tex = 12 * sin(wx * 41 + 3 * sin(wy * 29)) + 9 * sin(wy * 53 + wx * 7);
R = uint8(105 + tex);
G = uint8(118 + tex);
B = uint8(135 + tex);
R(red) = 140; G(red) = 8; B(red) = 8;
R(yel) = 220; G(yel) = 210; B(yel) = 30;
R(blu) = 20; G(blu) = 30; B(blu) = 220;
end
