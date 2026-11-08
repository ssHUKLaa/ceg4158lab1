% Run the Lab 3 analytical IK and parser for cases A, B, and C.
% Requires the uOttawa simAPI on the MATLAB path for ik_g3l_multiple.
addpath(fileparts(mfilename('fullpath')));
simApiPath = 'G:\pub\Courses\CEG4158\Matlab';
if isfolder(simApiPath)
    addpath(genpath(simApiPath));
end

% Creates pose, Qspec(:,:,1:3), and the reference joint configurations.
run('lab3_make_Qspec.m');
qAnlDeg = cell(1,3);
qParsedDeg = cell(1,3);
sourceRows = cell(1,3);
parserReports = cell(1,3);

for caseNumber = 1:3
    fprintf('\n========== Case %s ==========\n', caseNames{caseNumber});
    qAnlDeg{caseNumber} = ik_g3l_multiple(Qspec(:,:,caseNumber));
    [qParsedDeg{caseNumber}, sourceRows{caseNumber}, ...
        parserReports{caseNumber}] = parseGen3LiteIK( ...
        qAnlDeg{caseNumber}, Qspec(:,:,caseNumber));

    fprintf('Raw analytical solutions, degrees (one solution per row):\n');
    disp(qAnlDeg{caseNumber});
    fprintf('Parser diagnostics:\n');
    disp(parserReports{caseNumber});
    fprintf('Accepted source solution rows: %s\n', mat2str(sourceRows{caseNumber}.'));
    fprintf('Parsed valid joint configurations, degrees:\n');
    disp(qParsedDeg{caseNumber});
end

% To command an accepted parsed row in the simulator, convert degrees to
% radians first, then call: j.sendJointPositionCommand(qParsedDeg{case}(row,:)*pi/180)
