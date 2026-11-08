% Run the Lab 3 iterative IK cases and collect FK comparison data.
% Prerequisites: CoppeliaSim has the Gen3Lite fk scene open and running;
% MATLAB has a connected GEN3LITE object named j in the workspace.
% This script sends Cartesian commands, so run it only with the simulator.

addpath(fileparts(mfilename('fullpath')));
if ~exist('j','var') || isempty(j)
    error('Lab3Iterative:NoRobot', ...
        'Connect the robot first and leave its GEN3LITE object named j in the workspace.');
end

% Loads target poses, Qspec matrices, caseNames, and reference joint values.
run('lab3_make_Qspec.m');
settleSeconds = 5; % Increase if the arm has not reached a pose before reading it.

qIterRad = zeros(3,6);
qIterDeg = zeros(3,6);
QeffBaseIter = zeros(4,4,3);
positionErrorM = zeros(3,1);
orientationErrorDeg = zeros(3,1);

for k = 1:3
    fprintf('\nCase %s: commanding Cartesian pose ...\n', caseNames{k});
    j.sendCartesianPositionCommand(pose(k,:));
    pause(settleSeconds);

    q = j.JointPos;
    qIterRad(k,:) = q(:).';
    qIterDeg(k,:) = qIterRad(k,:) * 180/pi;
    QeffBaseIter(:,:,k) = gen3liteFK(qIterRad(k,:));

    positionErrorM(k) = norm(QeffBaseIter(1:3,4,k)-Qspec(1:3,4,k));
    Rerror = QeffBaseIter(1:3,1:3,k)' * Qspec(1:3,1:3,k);
    cosineAngle = (trace(Rerror)-1)/2;
    orientationErrorDeg(k) = acosd(max(-1,min(1,cosineAngle)));

    fprintf('Reference q (deg): %s\n', mat2str(qReferenceDeg(k,:),5));
    fprintf('Iterative q (deg): %s\n', mat2str(qIterDeg(k,:),5));
    fprintf('Qspec:\n'); disp(Qspec(:,:,k));
    fprintf('Qeff_base_iter from FK:\n'); disp(QeffBaseIter(:,:,k));
    fprintf('FK pose error: %.4f mm, %.4f deg\n', ...
        1000*positionErrorM(k), orientationErrorDeg(k));
    input(sprintf('Capture the CoppeliaSim screenshot for case %s, then press Enter: ', ...
        caseNames{k}), 's');
end

iterativeResults = table(caseNames(:), qIterDeg, ...
    1000*positionErrorM, orientationErrorDeg, ...
    'VariableNames', {'Case','q_iter_deg','PositionError_mm','OrientationError_deg'});
fprintf('\nIterative IK results for the report:\n');
disp(iterativeResults);

% Workspace outputs for the report:
% pose, Qspec, qReferenceDeg, qIterRad, qIterDeg, QeffBaseIter,
% positionErrorM, orientationErrorDeg, iterativeResults.
