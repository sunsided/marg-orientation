% Define symbolic variables
syms q0 q1 q2 q3 real  % quaternion components
syms x y z real        % components of the 3D vector

% Define the quaternion
q = [q0 q1 q2 q3];

% Define the 3D vector
v = [x; y; z];

% Down vetor.
% v(1) = 0;
% v(2) = 0;
% v(3) = 1;

% Quaternion multiplication (q*v*q')
q_conj = [q0, -q1, -q2, -q3];  % Conjugate of the quaternion

% Quaternion-vector multiplication q*v
v_quat = [0; v];  % Extend vector to quaternion form
qv = quat_mult(q, v_quat);  % Quaternion multiplication q*v

% Quaternion-vector multiplication (q*v)*q'
v_rot_quat = quat_mult(qv, q_conj);

% Extract rotated vector
v_rot = v_rot_quat(2:4);

% Compute the Jacobian of the rotated vector with respect to the quaternion
J = jacobian(v_rot, q);

% Display the Jacobian
disp('Jacobian of the rotated vector with respect to the quaternion:');
disp(J);

function res = quat_mult(q1, q2)
    % Quaternion multiplication: q1 * q2
    res = [q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
           q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
           q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
           q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)];
end
