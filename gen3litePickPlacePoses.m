function [poses, report] = gen3litePickPlacePoses(pick, place, qInitial, lift)
%GEN3LITEPICKPLACEPOSES Plan four tool positions in the DH base frame.
% pick/place are tool-tip targets in metres; lift defaults to 0.08 m.
if nargin < 4, lift = 0.08; end
validateattributes(lift, {'numeric'}, {'scalar','real','finite','positive'});
validateattributes(pick, {'numeric'}, {'real','finite','vector','numel',3});
validateattributes(place, {'numeric'}, {'real','finite','vector','numel',3});
pick = pick(:); place = place(:);
targets = [pick+[0;0;lift], pick, place+[0;0;lift], place];
poses = zeros(4,6);
report = repmat(struct('Success',false,'PositionError',inf, ...
    'Iterations',0,'ReachedPosition',zeros(1,3)), 4, 1);
seed = qInitial;
for k = 1:4
    [poses(k,:), report(k)] = gen3liteIK(targets(:,k), seed);
    seed = poses(k,:);
end
end
