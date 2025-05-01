function X = hand_eye_calibration(qEs, qSs, tEs, tSs)
% Input: 
%   qEs - 4xN matrix of quaternions representing robot hand rotations [w,x,y,z]
%   qSs - 4xN matrix of quaternions representing camera rotations [w,x,y,z]
%   tEs - 3xN matrix of robot hand translations
%   tSs - 3xN matrix of camera translations
% Output: 
%   X - 4x4 homogeneous transformation matrix (hand-eye transformation)

% Number of poses
N = size(qEs, 2);

% Convert quaternions to rotation matrices
TAs = zeros(4,4,N-1);
TBs = zeros(4,4,N-1);
for i = 1:N-1
    qE1 = qEs(:,i);
    qE2 = qEs(:,i+1);
    qS1 = qSs(:,i);
    qS2 = qSs(:,i+1);
    E1 = [quaternionToSO3(qE1), tEs(:,i); 0 0 0 1];
    E2 = [quaternionToSO3(qE2), tEs(:,i+1); 0 0 0 1];
    S1 = [quaternionToSO3(qS1), tSs(:,i); 0 0 0 1];
    S2 = [quaternionToSO3(qS2), tSs(:,i+1); 0 0 0 1];
    TAs(:,:,i) = E1\E2;
    TBs(:,:,i) = S1/S2;
end

% Solve for rotation part using quaternion approach
% AX = XB -> qA * qX = qX * qB
% This can be rewritten as: [qA]L * qX = [qB]R * qX
% where [q]L and [q]R are left and right quaternion multiplication matrices

% Construct the system matrix M
M = zeros(4*N-4, 4);
for i = 1:N-1
    % Get left multiplication matrix for qA
    TA = TAs(:,:,i);
    TB = TBs(:,:,i);
    qA = SO3ToQuaternion(TA(1:3,1:3));
    qB = SO3ToQuaternion(TB(1:3,1:3));
    sA = qA(1); vA = qA(2:4)';
    sB = qB(1); vB = qB(2:4)';

    M(4*i-3:4*i,:) = [sA - sB, -(vA - vB)';
                      (vA -vB), (sA-sB)*eye(3) + vecToso3(vA+vB)];
end

% Solve for quaternion using SVD
[~,~,V] = svd(M);
qX = V(:,end); % Last right singular vector

% Convert quaternion to rotation matrix
RX = quaternionToSO3(qX);

% Solve for translation part
% tA + RA*tX = RX*tB + tX
% This can be rewritten as: (I - RA)*tX = RX*tB - tA
A = zeros(3*N-3, 3);
b = zeros(3*N-3, 1);
for i = 1:N-1
    TA = TAs(:,:,i);
    TB = TBs(:,:,i);
    RA = TA(1:3,1:3);
    RB = TB(1:3,1:3);
    tA = TA(1:3,4);
    tB = TB(1:3,4);
    A(3*i-2:3*i,:) = RA - eye(3);
    b(3*i-2:3*i) = RX*tB - tA;
end

% Solve for translation using least squares
tX = pinv(A) * b;

% Construct the final transformation matrix
X = RandpToSE3(RX, tX);
end

