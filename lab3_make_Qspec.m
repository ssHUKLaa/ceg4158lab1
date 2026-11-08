% LAB3_MAKE_QSPEC Convert the three Lab 3 pose vectors into Q_spec matrices.
% Pose order: [x, y, z, alpha, beta, gamma]. Position is metres;
% Tait-Bryan angles are radians. Uses T = Trans * Rx * Ry * Rz.
% Requires gen3litePoseToT.m in this folder or on the MATLAB path.

addpath(fileparts(mfilename('fullpath')));

caseNames = {'A - home', 'B', 'C'};
pose = [ ...
    0.0570, -0.0100, 1.0033,  0.0000, 0.0000,  1.5708; ...
    0.2250,  0.2450, 0.5803,  0.0000, 1.5708, -1.5708; ...
    0.1701,  0.1481, 0.5742, -1.5708, 0.7854,  2.3562];

% These are the reference configurations printed in Lab 3 Figure 3,
% provided for comparison. They are not outputs of inverse kinematics.
qReferenceDeg = [ ...
     0,   0,   0,  0,   0, 0; ...
   -90,   0, -90,  0, -90, 0; ...
   -45,  45,  90,  0, -90, 0];

Qspec = zeros(4,4,size(pose,1));
for k = 1:size(pose,1)
    Qspec(:,:,k) = gen3litePoseToT(pose(k,:));
    fprintf('\nCase %s\n', caseNames{k});
    fprintf('Target pose [x y z alpha beta gamma]:\n');
    disp(pose(k,:));
    fprintf('Reference joint angles (degrees):\n');
    disp(qReferenceDeg(k,:));
    fprintf('Qspec =\n');
    disp(Qspec(:,:,k));
end

% Qspec(:,:,1), Qspec(:,:,2), Qspec(:,:,3) are the matrices for cases A, B, C.
% Example for the analytical IK call in the lab simAPI:
% qAnlDegA = ik_g3l_multiple(Qspec(:,:,1));
