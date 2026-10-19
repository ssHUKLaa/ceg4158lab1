% Lab 2: compare candidate DH forward kinematics with CoppeliaSim.
% Uses the exact scene-loading form given in the Lab 2 guide.
clear;
simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
coppeliaPath = ['C:\ProgramData\Microsoft\AppV\Client\Integration\' ...
    '6BF10E29-DF4B-4E33-AEDF-FB25B6B1B588\Root\VFS\ProgramFilesX64\' ...
    'CoppeliaRobotics\CoppeliaSimEdu'];
settleSeconds = 2;

% Lab-guide reference configurations, in degrees. These are within the
% approximate joint limits; q3=150 is near its upper limit.
names = {'zero','positive 90','negative 90','Kinova home ref', ...
    'retract','packaging'};
qTestDeg = [0,0,0,0,0,0; ...
            90,90,90,90,90,90; ...
           -90,-90,-90,-90,-90,-90; ...
             0,-16,75,0,-60,0; ...
            -3,21,150,-88,-40,-87; ...
           -90,148,148,-90,140,0];

addpath(genpath(simApiPath));
v = [];
j = [];
try
    v = VREP('path', coppeliaPath);
    v.loadscene('Gen3Lite fk', 'local', true);
    v.simstart();
    j = GEN3LITE(v, 'Gen3Lite');
    j.connect();

    positionError = zeros(size(qTestDeg,1),1);
    rotationErrorDeg = zeros(size(qTestDeg,1),1);
    for k = 1:size(qTestDeg,1)
        qCommand = qTestDeg(k,:) * pi/180;
        j.sendJointAndFingerPositionCommand(qCommand, [0]);
        pause(settleSeconds);

        % Compare using the measured joint values actually reached by the
        % simulator, not merely the requested values.
        measured = j.getJointAndFingerPos();
        qActual = measured(1:6);
        Tdh = gen3liteFK(qActual);
        Tsim = gen3litePoseToT(j.EndEffectorPose);

        positionError(k) = norm(Tdh(1:3,4)-Tsim(1:3,4));
        c = (trace(Tdh(1:3,1:3)'*Tsim(1:3,1:3))-1)/2;
        rotationErrorDeg(k) = acosd(max(-1,min(1,c)));

        fprintf('\n%s\n', names{k});
        fprintf('Commanded q (deg): %s\n', mat2str(qTestDeg(k,:),5));
        fprintf('Measured q  (deg): %s\n', mat2str(qActual*180/pi,5));
        fprintf('DH transform:\n'); disp(Tdh);
        fprintf('Simulator transform:\n'); disp(Tsim);
        fprintf('Position error: %.6g m; orientation error: %.6g deg\n', ...
            positionError(k), rotationErrorDeg(k));
    end

    summary = table(names(:), positionError, rotationErrorDeg, ...
        'VariableNames', {'Configuration','PositionError_m','OrientationError_deg'});
    fprintf('\nSummary (errors should approach zero after DH frames/offsets match):\n');
    disp(summary);
catch ME
    cleanupLab2(j, v);
    rethrow(ME);
end
cleanupLab2(j, v);

function cleanupLab2(j, v)
if ~isempty(j)
    try, j.disconnect(); catch, end
end
if ~isempty(v)
    try, v.simstop(); catch, end
    try, v.clearscene(); catch, end
    try, v.delete(); catch, end
end
end
