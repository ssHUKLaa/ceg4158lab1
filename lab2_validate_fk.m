% Lab 2: compare candidate DH forward kinematics with CoppeliaSim.
% By default, use a Gen3Lite fk scene opened manually in CoppeliaSim.
clear;
simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
coppeliaPath = ['C:\ProgramData\Microsoft\AppV\Client\Integration\' ...
    '6BF10E29-DF4B-4E33-AEDF-FB25B6B1B588\Root\VFS\ProgramFilesX64\' ...
    'CoppeliaRobotics\CoppeliaSimEdu'];
loadSceneFromMatlab = false; % Set true to use the guide's v.loadscene call.
clearSceneAtEnd = false;     % Keep the scene visible after the comparison.
if ~isfolder(simApiPath)
    error('Lab2FK:SimAPIPath', ...
        'simAPI folder not found: %s. Set simApiPath to the folder containing GEN3LITE.m.', ...
        simApiPath);
end
if loadSceneFromMatlab
    sceneFile = fullfile(simApiPath, 'scenes', 'Gen3Lite fk.ttt');
    if ~isfile(sceneFile)
        error('Lab2FK:ScenePath', ...
            'Lab scene not found: %s. Check that simApiPath points to the simAPI root.', ...
            sceneFile);
    end
end
if ~isfolder(coppeliaPath)
    error('Lab2FK:CoppeliaPath', ...
        'CoppeliaSim folder not found: %s. Set coppeliaPath to your installation folder.', ...
        coppeliaPath);
end
addpath(fileparts(mfilename('fullpath')));
addpath(genpath(simApiPath));
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

% Show the six substituted DH matrices and fixed tool-frame transform.
[TzeroTool, framesZero] = gen3liteFK(zeros(1,6));
Azero = zeros(4,4,6);
for i = 1:6
    % frames(:,:,i) is T0,i-1 and frames(:,:,i+1) is T0,i.
    Azero(:,:,i) = framesZero(:,:,i) \ framesZero(:,:,i+1);
end
fprintf('Candidate A_i matrices at q = [0 0 0 0 0 0]:\n');
for i = 1:6
    fprintf('A%d =\n', i);
    disp(Azero(:,:,i));
end
fprintf('DH chain T06 at zero pose:\n');
disp(framesZero(:,:,7));
fprintf('Base-to-tool transform T06*T6tool at zero pose:\n');
disp(TzeroTool);

v = [];
j = [];
try
    fprintf('Connecting to CoppeliaSim at %s ...\n', coppeliaPath);
    drawnow;
    v = VREP('path', coppeliaPath);
    if loadSceneFromMatlab
        fprintf('Loading Gen3Lite fk scene from simAPI ...\n');
        drawnow;
        v.loadscene('Gen3Lite fk', 'local', true);
    else
        fprintf(['Using the currently open scene. Confirm it is Gen3Lite fk ' ...
            'and that simulation is stopped.\n']);
        drawnow;
    end
    fprintf('Starting simulation ...\n');
    drawnow;
    v.simstart();
    fprintf('Connecting to the Gen3Lite model ...\n');
    drawnow;
    j = GEN3LITE(v, 'Gen3Lite');
    j.connect();
    fprintf('Running DH pose comparisons.\n');
    drawnow;

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
    cleanupLab2(j, v, clearSceneAtEnd);
    rethrow(ME);
end
cleanupLab2(j, v, clearSceneAtEnd);

function cleanupLab2(j, v, clearSceneAtEnd)
if ~isempty(j)
    try, j.disconnect(); catch, end
end
if ~isempty(v)
    try, v.simstop(); catch, end
    if clearSceneAtEnd
        try, v.clearscene(); catch, end
    end
    try, v.delete(); catch, end
end
end
