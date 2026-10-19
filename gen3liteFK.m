function [T, frames] = gen3liteFK(q)
%GEN3LITEFK Candidate DH tool pose in the DH base frame, q in radians.
% This uses the candidate classical-DH parameters documented in
% Lab2_FK_Preparation.md. Validate the base/tool frame choice against the
% Gen3Lite fk scene and j.EndEffectorPose before treating it as final.
% frames(:,:,i) is T0i; frames(:,:,1) is the base frame.
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',6});
q = double(q(:));
alpha = [pi/2; pi; pi/2; pi/2; pi/2; 0];
a = [0; 0.280; 0; 0; 0; 0];
d = [0.2433; 0.030; 0.020; 0.245; 0.057; 0.235];
offset = [0; pi/2; pi/2; pi/2; pi; pi/2];
frames = zeros(4,4,7);
frames(:,:,1) = eye(4);
for k = 1:6
    frames(:,:,k+1) = frames(:,:,k) * ...
        dhTransform(alpha(k), a(k), d(k), q(k)+offset(k));
end
T = frames(:,:,7);
end
