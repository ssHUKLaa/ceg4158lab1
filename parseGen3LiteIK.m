function [qParsedDeg, sourceRows, report] = parseGen3LiteIK(qAnlDeg, Qspec, options)
%PARSEGEN3LITEIK Filter analytical Gen3 Lite IK solutions.
%   [QPARSED, ROWS, REPORT] = PARSEGEN3LITEIK(QANLDEG, QSPEC) checks each
%   row of the N-by-6 analytical solution matrix. Angles in QANLDEG are
%   degrees; QSPEC is the desired 4-by-4 base-to-tool transform.
%
%   For each joint, it tries equivalent angles q + 360*k within the lab
%   joint limits, then validates the full pose using gen3liteFK (radians).
%   QPARSED contains only accepted rows in degrees; ROWS identifies their
%   original row numbers in QANLDEG. REPORT has one diagnostic row per input.
%
%   Optional fields in OPTIONS:
%     PositionToleranceM      default 1e-3
%     OrientationToleranceDeg default 0.1

if nargin < 3, options = struct(); end
if ~isnumeric(qAnlDeg) || ~isreal(qAnlDeg) || ndims(qAnlDeg) ~= 2 || ...
        size(qAnlDeg,2) ~= 6
    error('parseGen3LiteIK:SolutionsShape', ...
        'qAnlDeg must be a real numeric N-by-6 matrix in degrees.');
end
validateattributes(Qspec, {'numeric'}, {'real','finite','size',[4,4]});
if ~isstruct(options) || ~isscalar(options)
    error('parseGen3LiteIK:Options', 'options must be a scalar struct.');
end

positionTolerance = optionValue(options, 'PositionToleranceM', 1e-3);
orientationTolerance = optionValue(options, 'OrientationToleranceDeg', 0.1);
validateattributes(positionTolerance, {'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(orientationTolerance, {'numeric'}, ...
    {'scalar','real','finite','positive'});

limitsDeg = gen3liteJointLimits() * 180/pi;
nSolutions = size(qAnlDeg,1);
wrappedDeg = nan(nSolutions,6);
withinLimits = false(nSolutions,1);
poseMatches = false(nSolutions,1);
positionErrorM = nan(nSolutions,1);
orientationErrorDeg = nan(nSolutions,1);
accepted = false(nSolutions,1);
reason = repmat({''},nSolutions,1);

for row = 1:nSolutions
    raw = double(qAnlDeg(row,:));
    if any(~isfinite(raw))
        reason{row} = 'Non-finite analytical solution';
        continue;
    end

    candidate = nan(1,6);
    limitFailure = false;
    for joint = 1:6
        lo = limitsDeg(joint,1);
        hi = limitsDeg(joint,2);
        % Find all integer 360-degree shifts that place this angle in range.
        kMin = ceil((lo - raw(joint))/360 - 1e-12);
        kMax = floor((hi - raw(joint))/360 + 1e-12);
        if kMin > kMax
            limitFailure = true;
            break;
        end
        equivalents = raw(joint) + 360*(kMin:kMax);
        % The Gen3 Lite ranges are narrower than 360 degrees. If an option
        % yields more than one equivalent, choose the one nearest zero.
        [~, nearest] = min(abs(equivalents));
        candidate(joint) = equivalents(nearest);
    end
    if limitFailure
        reason{row} = 'No 360-degree equivalent satisfies all joint limits';
        continue;
    end

    wrappedDeg(row,:) = candidate;
    withinLimits(row) = true;
    Tfk = gen3liteFK(candidate*pi/180);
    positionErrorM(row) = norm(Tfk(1:3,4)-Qspec(1:3,4));
    relativeRotation = Tfk(1:3,1:3)' * Qspec(1:3,1:3);
    cosineAngle = (trace(relativeRotation)-1)/2;
    orientationErrorDeg(row) = acosd(max(-1,min(1,cosineAngle)));

    poseMatches(row) = positionErrorM(row) <= positionTolerance && ...
        orientationErrorDeg(row) <= orientationTolerance;
    accepted(row) = poseMatches(row);
    if accepted(row)
        reason{row} = 'Accepted';
    elseif positionErrorM(row) > positionTolerance && ...
            orientationErrorDeg(row) > orientationTolerance
        reason{row} = 'FK position and orientation do not match Qspec';
    elseif positionErrorM(row) > positionTolerance
        reason{row} = 'FK position does not match Qspec';
    else
        reason{row} = 'FK orientation does not match Qspec';
    end
end

sourceRows = find(accepted);
qParsedDeg = wrappedDeg(sourceRows,:);
report = table((1:nSolutions)', wrappedDeg, withinLimits, ...
    positionErrorM, orientationErrorDeg, poseMatches, accepted, reason, ...
    'VariableNames', {'Solution','WrappedAnglesDeg','WithinJointLimits', ...
    'PositionError_m','OrientationError_deg','PoseMatches','Accepted','Reason'});
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
