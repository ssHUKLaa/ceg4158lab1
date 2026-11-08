function [T, frames, A] = gen3liteFK(q)
%GEN3LITEFK Candidate base-to-tool pose from classical DH, q in radians.
% The six A matrices produce T06. A fixed T6tool rotation maps the terminal
% DH frame to the simulator tool frame, inferred from the supplied pose data.
% frames(:,:,i) contains cumulative DH transforms T0i, before T6tool.
validateattributes(q, {'numeric'}, {'real','finite','vector','numel',6});
q = double(q(:));
alpha = [pi/2; pi; pi/2; pi/2; pi/2; 0];
a = [0; 0.280; 0; 0; 0; 0];
d = [0.2433; 0.030; 0.020; 0.245; 0.057; 0.235];
offset = [0; pi/2; pi/2; pi/2; pi; pi/2];
frames = zeros(4,4,7);
A = zeros(4,4,6);
frames(:,:,1) = eye(4);
for k = 1:6
    A(:,:,k) = dhTransform(alpha(k), a(k), d(k), q(k)+offset(k));
    frames(:,:,k+1) = frames(:,:,k) * A(:,:,k);
end
T06 = frames(:,:,7);
T6tool = dhTransform(0, 0, 0, pi/2);
T = T06 * T6tool;
end
