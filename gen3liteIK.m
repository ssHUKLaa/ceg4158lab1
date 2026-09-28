function [q, info] = gen3liteIK(targetPosition, qInitial, options)
%GEN3LITEIK Position IK using damped least squares and joint limit projection.
%   q = gen3liteIK(targetPosition, qInitial) uses metres and radians in
%   the DH base frame. q is a 1-by-6 row vector for the lab simAPI.
%   This solves position only; wrist orientation is unconstrained.
%   Optional fields: Tolerance (m), MaxIterations, Damping, MaxStep (rad).
if nargin < 3, options = struct(); end
validateattributes(targetPosition, {'numeric'}, ...
    {'real','finite','vector','numel',3});
validateattributes(qInitial, {'numeric'}, ...
    {'real','finite','vector','numel',6});
if ~isstruct(options) || ~isscalar(options)
    error('gen3liteIK:Options', 'options must be a scalar struct.');
end
tol = optionValue(options, 'Tolerance', 1e-3);
maxIter = optionValue(options, 'MaxIterations', 300);
lambda = optionValue(options, 'Damping', 0.03);
maxStep = optionValue(options, 'MaxStep', 0.15);
validateattributes(tol, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(maxIter, {'numeric'}, {'scalar','real','finite','integer','positive'});
validateattributes(lambda, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(maxStep, {'numeric'}, {'scalar','real','finite','positive'});
target = double(targetPosition(:));
limits = gen3liteJointLimits();
q = min(max(double(qInitial(:)), limits(:,1)), limits(:,2));
bestQ = q;
bestError = inf;
iterations = 0;
for k = 1:maxIter
    iterations = k;
    [T, frames] = gen3liteFK(q);
    e = target - T(1:3,4);
    err = norm(e);
    if err < bestError
        bestError = err;
        bestQ = q;
    end
    if err <= tol, break; end
    J = zeros(3,6);
    for i = 1:6
        axis = frames(1:3,3,i);
        origin = frames(1:3,4,i);
        J(:,i) = cross(axis, T(1:3,4)-origin);
    end
    step = J' * ((J*J' + lambda^2*eye(3)) \ e);
    stepNorm = norm(step);
    if stepNorm > maxStep, step = step * (maxStep/stepNorm); end
    % Backtracking avoids making a difficult target worse at a joint limit.
    accepted = false;
    scale = 1;
    for trial = 1:12
        candidate = min(max(q + scale*step, limits(:,1)), limits(:,2));
        Tc = gen3liteFK(candidate);
        if norm(target - Tc(1:3,4)) < err
            q = candidate;
            accepted = true;
            break;
        end
        scale = scale/2;
    end
    if ~accepted, break; end
end
q = bestQ.';
info = struct('Success', bestError <= tol, ...
    'PositionError', bestError, 'Iterations', iterations, ...
    'ReachedPosition', gen3liteFKPosition(bestQ));
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name), value = options.(name);
else, value = defaultValue;
end
end

function p = gen3liteFKPosition(q)
T = gen3liteFK(q);
p = T(1:3,4).';
end
