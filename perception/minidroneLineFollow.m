function [trajectory, trackView] = minidroneLineFollow(R, G, B, pos)
%MINIDRONELINEFOLLOW Red-line follower and landing logic for the PX4 minidrone.
%#codegen
% Inputs:  R, G, B  120x160 uint8 planes from the downward camera
%          pos      [X; Y; Z; yaw] in the guidance frame (NED: Z negative up,
%                   yaw in rad, clockwise from +X)
% Outputs: trajectory  [x; y; Z; yawCmd] for the localToNED block. localToNED
%                      rotates (x, y) by yawCmd, so the target point is
%                      pre-rotated by -yawCmd here.
%          trackView   120x160x3 uint8 image for the Tracking View window
%
% Following: a ring of samples around the vehicle's ground point finds where
% the line crosses it. The crossing nearest the travel direction (ignoring
% the one we came from) is the way ahead; the vehicle yaws to face it and
% flies toward that point on the line.
% Landing: a red region wide enough to hold a 0.16 m disk (wider than the
% 0.1 m line) with at most one line leaving it is the landing circle.

TAKEOFF = 1;
FOLLOW = 2;
GAP = 3;
APPROACH = 4;
LAND = 5;
SEARCH = 6;

CRUISE_Z = -1.1;
LINE_W = 0.10;
RING_PX = 48;
STEP = 0.30;
TURN_STEP = 0.12;
GAP_STEP = 0.12;
APPROACH_STEP = 0.20;
LEAD = 0.10;
MAX_GAP = 0.8;
YAW_STEP = 0.25;
YAW_DEADBAND = 0.05;
LOCK_DIST = 0.04;

cam = minidroneCameraModel();

x = double(pos(1));
y = double(pos(2));
z = double(pos(3));
yaw = double(pos(4));

persistent mode dirX dirY homeX homeY homeYaw gapX gapY lockX lockY ...
    yawHold lostCount searchK searchTX searchTY poseHist
if isempty(poseHist)
    poseHist = repmat([x; y; z; yaw], 1, cam.lagSteps + 1);
end
% The frame arriving now was taken lagSteps guidance steps ago; read it with
% the pose from then, or every heading change makes the line look tilted.
poseHist = [poseHist(:, 2:end), [x; y; z; yaw]];
xi = poseHist(1, 1);
yi = poseHist(2, 1);
zi = poseHist(3, 1);
yawi = poseHist(4, 1);

if isempty(mode)
    mode = TAKEOFF;
    dirX = cos(yaw);
    dirY = sin(yaw);
    homeX = x;
    homeY = y;
    homeYaw = yaw;
    gapX = x;
    gapY = y;
    lockX = x;
    lockY = y;
    yawHold = yaw;
    lostCount = 0;
    searchK = 0;
    searchTX = x;
    searchTY = y;
end

height = max(0.15, -zi + cam.heightOffset);
mpp = height * cam.tanHalfHfov / (cam.cols / 2);
mask = redMask(R, G, B);
vr = (cam.rows + 1) / 2;
vc = (cam.cols + 1) / 2 - cam.camRight / (cam.colRight * mpp);
ringM = RING_PX * mpp;

[runAng, runWid, nRuns, ringHit, ringIn, ringR, ringC] = ringRuns(mask, vr, vc, RING_PX, cam);
[diskFound, diskFull, dr, dc, core] = findDisk(mask, mpp, cam);
[dX, dY] = pixelToWorld(dr, dc, xi, yi, yawi, mpp, cam);

travel = atan2(dirY, dirX);
[hasCand, candAng, hasIn, inAng, split] = chooseBranch(runAng, runWid, nRuns, yawi, travel, ...
    LINE_W / (RING_PX * mpp));

% World points where the line leaves (cand) and enters (in) the ring.
candX = xi + ringM * cos(candAng);
candY = yi + ringM * sin(candAng);
lineAng = candAng;
if hasCand && hasIn && ~split && abs(wrapPi(candAng - inAng - pi)) < 0.6
    % Straight section: head along the chord through both crossings. It is
    % the line's own direction, so being off to one side does not yaw the
    % vehicle (the aim point below pulls it back onto the line instead).
    lineAng = atan2(candY - (yi + ringM * sin(inAng)), candX - (xi + ringM * cos(inAng)));
end

xc = x;
yc = y;
zc = CRUISE_Z;

if mode == TAKEOFF
    xc = homeX;
    yc = homeY;
    yawHold = homeYaw;
    if -z > 0.85 * -CRUISE_Z
        dirX = cos(yaw);
        dirY = sin(yaw);
        if nRuns > 0
            mode = FOLLOW;
        else
            mode = SEARCH;
            searchK = 0;
            searchTX = x;
            searchTY = y;
        end
    end
end

if diskFound && (mode == FOLLOW || mode == GAP || mode == SEARCH)
    mode = APPROACH;
    lostCount = 0;
end

if mode == SEARCH
    if hasCand
        mode = FOLLOW;
    else
        if hypot(x - searchTX, y - searchTY) < 0.08
            searchK = searchK + 1;
            leg = 0.35 * ceil(searchK / 2);
            side = mod(searchK - 1, 4);
            if side == 0
                searchTX = searchTX + leg;
            elseif side == 1
                searchTY = searchTY + leg;
            elseif side == 2
                searchTX = searchTX - leg;
            else
                searchTY = searchTY - leg;
            end
        end
        [xc, yc] = clampStep(x, y, searchTX, searchTY, 0.25);
    end
end

if mode == GAP && hasCand
    mode = FOLLOW;
end

if mode == FOLLOW
    if hasCand
        dirX = cos(lineAng);
        dirY = sin(lineAng);
        turnErr = wrapPi(lineAng - yawHold);
        step = STEP;
        aimX = candX + LEAD * dirX;
        aimY = candY + LEAD * dirY;
        if abs(wrapPi(lineAng - yaw)) > 0.5
            % Pivot near the corner instead of cutting across it.
            aimX = xi + 0.5 * ringM * dirX;
            aimY = yi + 0.5 * ringM * dirY;
            step = TURN_STEP;
        end
        [xc, yc] = clampStep(x, y, aimX, aimY, step);
        if abs(turnErr) > YAW_DEADBAND
            yawHold = wrapPi(yawHold + max(-YAW_STEP, min(YAW_STEP, turnErr)));
        end
    else
        mode = GAP;
        gapX = x;
        gapY = y;
    end
end

if mode == GAP
    if hypot(x - gapX, y - gapY) > MAX_GAP
        % Paint ended and no circle turned up: land here rather than wander.
        mode = LAND;
        lockX = x;
        lockY = y;
    else
        xc = x + GAP_STEP * dirX;
        yc = y + GAP_STEP * dirY;
    end
end

if mode == APPROACH
    if diskFound
        lostCount = 0;
        [xc, yc] = clampStep(x, y, dX, dY, APPROACH_STEP);
        if diskFull && hypot(dX - x, dY - y) < LOCK_DIST
            mode = LAND;
            lockX = dX;
            lockY = dY;
        end
    else
        lostCount = lostCount + 1;
        if lostCount > 3
            mode = FOLLOW;
        end
    end
end

if mode == LAND
    if diskFound && diskFull && height > 0.5
        lockX = lockX + 0.3 * (dX - lockX);
        lockY = lockY + 0.3 * (dY - lockY);
    end
    xc = lockX;
    yc = lockY;
    if height > 0.35
        zc = min(0.05, z + 0.12);
    else
        zc = min(0.05, z + 0.06);
    end
end

xc = min(3.4, max(-0.4, xc));
yc = min(2.9, max(-0.9, yc));

yawCmd = yawHold;
c = cos(yawCmd);
s = sin(yawCmd);
trajectory = zeros(4, 1);
trajectory(1) = xc * c + yc * s;
trajectory(2) = -xc * s + yc * c;
trajectory(3) = zc;
trajectory(4) = yawCmd;

trackView = drawTrackView(R, G, B, mask, core, ringHit, ringIn, ringR, ringC, ...
    hasCand, wrapPi(candAng - yawi), hasIn, wrapPi(inAng - yawi), diskFound, dr, dc, ...
    vr, vc, RING_PX, mode, cam);
end

function mask = redMask(R, G, B)
r = int16(R);
g = int16(G);
b = int16(B);
mask = (R >= uint8(80)) & ((r - g) >= int16(50)) & ((r - b) >= int16(50));
end

function [ang, wid, n, hit, inside, rr, cc] = ringRuns(mask, vr, vc, radPx, cam)
% Body-frame angles (0 = forward, +pi/2 = right) of line crossings on a ring.
N = 96;
ang = zeros(1, 16);
wid = zeros(1, 16);
n = 0;
hit = false(1, N);
inside = false(1, N);
rr = zeros(1, N);
cc = zeros(1, N);
for k = 1:N
    th = 2 * pi * (k - 1) / N;
    for rad = [radPx, radPx - 4]
        r = round(vr - cam.rowFwd * rad * cos(th));
        c = round(vc + cam.colRight * rad * sin(th));
        onImage = r >= 1 && r <= cam.rows && c >= 1 && c <= cam.cols;
        if rad == radPx
            rr(k) = r;
            cc(k) = c;
            inside(k) = onImage;
        end
        if onImage
            if mask(r, c)
                hit(k) = true;
            end
        end
    end
end
[ang, wid, n] = circularRuns(hit, N, ang, wid);
end

function [ang, wid, n] = circularRuns(hit, N, ang, wid)
n = 0;
if ~any(hit)
    return
end
if all(hit)
    n = 1;
    ang(1) = 0;
    wid(1) = 2 * pi;
    return
end
k0 = 1;
while hit(k0)
    k0 = k0 + 1;
end
inRun = false;
runStart = 0;
for t = 1:N
    k = mod(k0 - 1 + t, N) + 1;
    if hit(k)
        if ~inRun
            inRun = true;
            runStart = t;
        end
    elseif inRun
        inRun = false;
        len = t - runStart;
        if len >= 2 && n < numel(ang)
            mid = runStart + (len - 1) / 2;
            n = n + 1;
            ang(n) = wrapPi(2 * pi * (k0 - 1 + mid) / N);
            wid(n) = 2 * pi * len / N;
        end
    end
end
end

function [hasCand, candAng, hasIn, inAng, split] = chooseBranch(runAng, runWid, nRuns, yaw, travel, expW)
% World-frame angles. The run nearest straight behind is where we came from;
% of the rest, take the one nearest the current travel direction.
hasCand = false;
candAng = travel;
hasIn = false;
inAng = travel + pi;
split = false;
if nRuns == 0
    return
end
behind = travel + pi;
w = zeros(1, 16);
bestIn = 0;
bestInDiff = 100 * pi / 180;
for i = 1:nRuns
    w(i) = wrapPi(yaw + runAng(i));
    dd = abs(wrapPi(w(i) - behind));
    if dd < bestInDiff
        bestInDiff = dd;
        bestIn = i;
    end
end
cand = zeros(1, 17);
nC = 0;
for i = 1:nRuns
    if i == bestIn
        hasIn = true;
        inAng = w(i);
        if runWid(i) > 2.0 * expW
            % A sharp turn's outgoing line can merge with the incoming one on
            % the ring. Split the run into two line-width parts.
            a = wrapPi(w(i) - runWid(i) / 2 + expW / 2);
            b = wrapPi(w(i) + runWid(i) / 2 - expW / 2);
            if abs(wrapPi(a - behind)) <= abs(wrapPi(b - behind))
                inAng = a;
                other = b;
            else
                inAng = b;
                other = a;
            end
            nC = nC + 1;
            cand(nC) = other;
            split = true;
        end
    else
        nC = nC + 1;
        cand(nC) = w(i);
    end
end
best = inf;
for j = 1:nC
    dd = abs(wrapPi(cand(j) - travel));
    if dd < best
        best = dd;
        candAng = cand(j);
        hasCand = true;
    end
end
end

function [found, full, cr, cc, core] = findDisk(mask, mpp, cam)
% Landing circle: 0.2 m diameter vs. 0.1 m line. Pixels at least 0.08 m from
% any non-red pixel only exist inside the circle (and inside V corners
% sharper than 30 deg, which the rules exclude).
found = false;
full = false;
cr = (cam.rows + 1) / 2;
cc = (cam.cols + 1) / 2;
% Pad with background: red cut off by the image border must not look wide.
padded = false(cam.rows + 2, cam.cols + 2);
padded(2:end-1, 2:end-1) = mask;
D = bwdist(~padded);
core = D(2:end-1, 2:end-1) >= 0.08 / mpp;
if ~any(core(:))
    return
end
[L, nLab] = bwlabel(core, 8);
MAXLAB = 8;
cnt = zeros(1, MAXLAB);
for k = 1:numel(L)
    lab = L(k);
    if lab >= 1 && lab <= MAXLAB
        cnt(lab) = cnt(lab) + 1;
    end
end
[~, best] = max(cnt(1:min(nLab, MAXLAB)));
sr = 0;
sc = 0;
rmin = cam.rows;
rmax = 1;
cmin = cam.cols;
cmax = 1;
for c = 1:cam.cols
    for r = 1:cam.rows
        if L(r, c) == best
            sr = sr + r;
            sc = sc + c;
            rmin = min(rmin, r);
            rmax = max(rmax, r);
            cmin = min(cmin, c);
            cmax = max(cmax, c);
        end
    end
end
cr = sr / cnt(best);
cc = sc / cnt(best);
if max(rmax - rmin, cmax - cmin) * mpp > 0.09
    return
end

rIn = 0.085 / mpp;
rA1 = 0.125 / mpp;
rA2 = 0.16 / mpp;
inTot = 0;
inRed = 0;
annTot = 0;
annRed = 0;
r0 = max(1, floor(cr - rA2));
r1 = min(cam.rows, ceil(cr + rA2));
c0 = max(1, floor(cc - rA2));
c1 = min(cam.cols, ceil(cc + rA2));
for c = c0:c1
    for r = r0:r1
        d = hypot(r - cr, c - cc);
        if d <= rIn
            inTot = inTot + 1;
            inRed = inRed + double(mask(r, c));
        elseif d >= rA1 && d <= rA2
            annTot = annTot + 1;
            annRed = annRed + double(mask(r, c));
        end
    end
end
if inTot < 0.9 * pi * rIn ^ 2 || annTot < 0.6 * pi * (rA2 ^ 2 - rA1 ^ 2)
    return
end
if inRed < 0.88 * inTot || annRed > 0.35 * annTot
    return
end

% Count lines leaving the circle: at most one (the track's last segment).
N = 72;
hit = false(1, N);
inside = 0;
rad = (rA1 + rA2) / 2;
for k = 1:N
    th = 2 * pi * (k - 1) / N;
    r = round(cr - rad * cos(th));
    c = round(cc + rad * sin(th));
    if r >= 1 && r <= cam.rows && c >= 1 && c <= cam.cols
        inside = inside + 1;
        hit(k) = mask(r, c);
    end
end
[~, ~, nCross] = circularRuns(hit, N, zeros(1, 16), zeros(1, 16));
if nCross > 1
    return
end
found = true;
full = inTot >= 0.97 * pi * rIn ^ 2 && inside == N;
end

function [wx, wy] = pixelToWorld(r, c, x, y, yaw, mpp, cam)
fwd = cam.rowFwd * ((cam.rows + 1) / 2 - r) * mpp;
right = cam.colRight * (c - (cam.cols + 1) / 2) * mpp + cam.camRight;
wx = x + fwd * cos(yaw) - right * sin(yaw);
wy = y + fwd * sin(yaw) + right * cos(yaw);
end

function [x, y] = clampStep(x0, y0, x1, y1, maxStep)
dx = x1 - x0;
dy = y1 - y0;
d = hypot(dx, dy);
if d > maxStep
    x = x0 + maxStep * dx / d;
    y = y0 + maxStep * dy / d;
else
    x = x1;
    y = y1;
end
end

function a = wrapPi(a)
a = mod(a + pi, 2 * pi) - pi;
end

function img = drawTrackView(R, G, B, mask, core, ringHit, ringIn, ringR, ringC, ...
    hasCand, candBody, hasIn, inBody, diskFound, dr, dc, vr, vc, radPx, mode, cam)
% Dimmed camera + red detections, ring samples, chosen branch and mode bar.
gray = uint8((0.30 * double(R) + 0.59 * double(G) + 0.11 * double(B)) * 0.45);
p1 = gray;
p2 = gray;
p3 = gray;
p1(mask) = 255;
p2(mask) = 40;
p3(mask) = 40;
p1(core) = 255;
p2(core) = 230;
p3(core) = 0;
for k = 1:numel(ringHit)
    if ringIn(k)
        r = ringR(k);
        c = ringC(k);
        if ringHit(k)
            p1(r, c) = 0; p2(r, c) = 255; p3(r, c) = 255;
        else
            p1(r, c) = 0; p2(r, c) = 90; p3(r, c) = 90;
        end
    end
end
if hasIn
    [p1, p2, p3] = drawBox(p1, p2, p3, vr - cam.rowFwd * radPx * cos(inBody), ...
        vc + cam.colRight * radPx * sin(inBody), 2, 255, 140, 0, cam);
end
if hasCand
    for t = 4:4:radPx
        [p1, p2, p3] = drawBox(p1, p2, p3, vr - cam.rowFwd * t * cos(candBody), ...
            vc + cam.colRight * t * sin(candBody), 0, 0, 255, 0, cam);
    end
    [p1, p2, p3] = drawBox(p1, p2, p3, vr - cam.rowFwd * radPx * cos(candBody), ...
        vc + cam.colRight * radPx * sin(candBody), 3, 0, 255, 0, cam);
end
if diskFound
    [p1, p2, p3] = drawBox(p1, p2, p3, dr, dc, 3, 255, 0, 255, cam);
end
[p1, p2, p3] = drawBox(p1, p2, p3, vr, vc, 1, 255, 255, 255, cam);

modeColor = [255 255 255; 0 255 0; 255 140 0; 255 230 0; 255 0 255; 60 120 255];
for c = 1:6 * mode
    for r = 1:5
        p1(r, c) = modeColor(mode, 1);
        p2(r, c) = modeColor(mode, 2);
        p3(r, c) = modeColor(mode, 3);
    end
end
img = cat(3, p1, p2, p3);
end

function [p1, p2, p3] = drawBox(p1, p2, p3, r, c, half, cr, cg, cb, cam)
r = round(r);
c = round(c);
for rr = r - half:r + half
    for cc = c - half:c + half
        if rr >= 1 && rr <= cam.rows && cc >= 1 && cc <= cam.cols
            p1(rr, cc) = cr;
            p2(rr, cc) = cg;
            p3(rr, cc) = cb;
        end
    end
end
end
