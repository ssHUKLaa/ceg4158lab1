% Connect MATLAB to the Gen3Lite in the scene already open in CoppeliaSim.
% Run this script from MATLAB's base workspace before lab3_run_iterative_ik.
% It intentionally leaves v and j connected and available in the workspace.

simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
coppeliaPath = ['C:\ProgramData\Microsoft\AppV\Client\Integration\' ...
    '6BF10E29-DF4B-4E33-AEDF-FB25B6B1B588\Root\VFS\ProgramFilesX64\' ...
    'CoppeliaRobotics\CoppeliaSimEdu'];

if ~isfolder(simApiPath)
    error('Lab3Connect:SimAPIPath', ...
        'simAPI folder not found: %s. Update simApiPath to the folder containing VREP.m.', ...
        simApiPath);
end
if ~isfolder(coppeliaPath)
    error('Lab3Connect:CoppeliaPath', ...
        'CoppeliaSim folder not found: %s. Update coppeliaPath to your installation folder.', ...
        coppeliaPath);
end

addpath(fileparts(mfilename('fullpath')));
addpath(genpath(simApiPath));

% The scene is already open manually in CoppeliaSim; do not load another scene.
v = VREP('path', coppeliaPath);
v.simstart();
j = GEN3LITE(v, 'Gen3Lite');
j.connect();

jointAndFingerPosition = j.getJointAndFingerPos();
fprintf('Connected to Gen3Lite. Current joint/finger positions:\n');
disp(jointAndFingerPosition);
