% Example for MATLAB R2024b and the uOttawa simAPI.
% The lab guide's examples supply six joint angles directly. This IK example
% instead needs a target tool-tip position to calculate those joint angles.
% The guide does not provide block coordinates or a verified DH base pose.
% Task: move the green cylinder onto the red block. Open that scene manually
% in CoppeliaSim before running. Set measured values below. Metres throughout.
simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
coppeliaPath = 'C:\ProgramData\Microsoft\AppV\Client\Integration\6BF10E29-DF4B-4E33-AEDF-FB25B6B1B588\Root\VFS\ProgramFilesX64\CoppeliaRobotics\CoppeliaSimEdu';  % Lab workstation default.
executeMotion = false;           % Set true after checking targets and poses.
cylinderCenterWorld = [NaN, NaN, NaN]; % Centre of green cylinder.
redBlockCenterWorld = [NaN, NaN, NaN]; % Centre of red block.
cylinderHeight = NaN;            % Full height of green cylinder.
redBlockHeight = NaN;            % Full height of red block.
toolToGraspZ = 0;                % Tool-frame Z minus cylinder grasp-centre Z.
baseWorld = [0, 0, 0.052];        % Verify base origin/axes in your scene.
lift = 0.08;
settleSeconds = 3;               % Adjust for observed simulator motion.
gripClosure = 100;               % Tune if the cylinder slips or is displaced.

if ~executeMotion
    fprintf(['Measure the green cylinder and red block in CoppeliaSim, ' ...
        'verify the DH-to-world transform, then set executeMotion=true.\n']);
    return;
end
if any(~isfinite([cylinderCenterWorld, redBlockCenterWorld, ...
        cylinderHeight, redBlockHeight, toolToGraspZ, baseWorld])) || ...
        cylinderHeight <= 0 || redBlockHeight <= 0
    error('Fill in measured cylinder/block positions and heights.');
end
% Grasp at the cylinder centre. At placement, its bottom meets the block top.
pickWorld = cylinderCenterWorld + [0, 0, toolToGraspZ];
placeWorld = [redBlockCenterWorld(1:2), ...
    redBlockCenterWorld(3) + redBlockHeight/2 + ...
    cylinderHeight/2 + toolToGraspZ];
addpath(genpath(simApiPath));
v = [];
j = [];
try
    v = VREP('path', coppeliaPath);
    % The Gen3Lite blocks scene is already open manually in CoppeliaSim.
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
    j.sendFingerPositionCommand([gripClosure]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(1,:), [gripClosure]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(3,:), [gripClosure]);
    pause(settleSeconds);
    j.sendJointAndFingerPositionCommand(poses(4,:), [gripClosure]);
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
    try, v.delete(); catch, end
end
end
