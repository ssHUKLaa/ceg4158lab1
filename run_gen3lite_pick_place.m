% Example for MATLAB R2024b and the uOttawa simAPI.
% Measure the block and destination tool-tip positions in the scene first.
% Set these values before enabling executeMotion. All distances are metres.
simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
coppeliaPath = 'COPPELIASIM_PATH';  % Replace with installed CoppeliaSim path.
executeMotion = false;           % Set true after checking targets and poses.
pickWorld = [NaN, NaN, NaN];      % Desired tool-tip position at grasp.
placeWorld = [NaN, NaN, NaN];     % Desired tool-tip position at release.
baseWorld = [0, 0, 0.052];        % Verify base origin/axes in your scene.
lift = 0.08;
settleSeconds = 3;               % Adjust for observed simulator motion.

if ~executeMotion
    fprintf(['Set CoppeliaSim path, measure world-frame targets, verify the ' ...
        'DH-to-world transform, then set executeMotion=true.\n']);
    return;
end
if strcmp(coppeliaPath, 'COPPELIASIM_PATH') || ...
        any(~isfinite([pickWorld, placeWorld, baseWorld]))
    error('Fill in coppeliaPath, pickWorld, placeWorld, and baseWorld.');
end
addpath(genpath(simApiPath));
v = [];
j = [];
try
    v = VREP('path', coppeliaPath);
    v.loadscene('Gen3Lite blocks');
    v.simstart();
    j = GEN3LITE(v, 'Gen3Lite');
    j.connect();
    j.goToHomePosition();
    pause(settleSeconds);
    current = j.getJointAndFingerPos();
    qInitial = current(1:6);

    % Translation-only example: verify that DH and world axes agree.
    pickBase = pickWorld - baseWorld;
    placeBase = placeWorld - baseWorld;
    [poses, report] = gen3litePickPlacePoses( ...
        pickBase, placeBase, qInitial, lift);
    if ~all([report.Success])
        error('IK failed: position errors (m): %s', ...
            mat2str([report.PositionError], 4));
    end
    disp('Joint configurations, degrees:');
    disp(poses * 180/pi);

    j.sendJointAndFingerPositionCommand(poses(1,:), [0]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(2,:), [0]);
    pause(settleSeconds);
    j.sendFingerPositionCommand([100]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(1,:), [100]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(3,:), [100]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(4,:), [100]);
    pause(settleSeconds);
    j.sendFingerPositionCommand([0]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(3,:), [0]);
    pause(settleSeconds);
    j.goToHomePosition();
catch ME
    cleanupRobot(j, v);
    rethrow(ME);
end
cleanupRobot(j, v);

function cleanupRobot(j, v)
if ~isempty(j)
    try, j.disconnect(); catch, end
end
if ~isempty(v)
    try, v.simstop(); catch, end
    try, v.clearscene(); catch, end
    try, v.delete(); catch, end
end
end
