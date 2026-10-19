function T = gen3litePoseToT(pose)
%GEN3LITEPOSETOT Convert simAPI EndEffectorPose to a 4-by-4 transform.
% pose = [x y z alpha beta gamma], metres and radians. Lab 2 specifies
% Q = QT(x,y,z) * QRx(alpha) * QRy(beta) * QRz(gamma).
validateattributes(pose, {'numeric'}, {'real','finite','vector','numel',6});
pose = double(pose(:));
alpha = pose(4); beta = pose(5); gamma = pose(6);
Rx = [1, 0, 0; 0, cos(alpha), -sin(alpha); 0, sin(alpha), cos(alpha)];
Ry = [cos(beta), 0, sin(beta); 0, 1, 0; -sin(beta), 0, cos(beta)];
Rz = [cos(gamma), -sin(gamma), 0; sin(gamma), cos(gamma), 0; 0, 0, 1];
T = [Rx*Ry*Rz, pose(1:3); 0, 0, 0, 1];
end
