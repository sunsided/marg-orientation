% pkg load symbolic
clc

% Accelerometer
syms a_x a_y a_z
assume([a_x a_y a_z], 'real')
% syms a11 a12 a13 a21 a22 a23 a31 a32 a33
% A = [a11, a12, a13; a21, a22, a23; a31, a32, a33];

% Magnetometer
syms m_x m_y m_z
assume([m_x m_y m_z], 'real')
% syms m11 m12 m13 m21 m22 m23 m31 m32 m33
% M = [m11, m12, m13; m21, m22, m23; m31, m32, m33];

% Obtain the noise covariances
syms sa11 sa22 sa33 sm11 sm22 sm33;
assume([sa11 sa22 sa33 sm11 sm22 sm33], 'real')
R_acc = [sa11, 0, 0; 0, sa22, 0; 0, 0, sa33]
R_mag = [sm11, 0, 0; 0, sm22, 0; 0, 0, sm33]

% Measurement vectors - assume normalized!
a = [a_x; a_y; a_z];
m = [m_x; m_y; m_z];

% TRIAD vectors
disp("Building TRIAD vectors");
b1 = a
b2 = (m - dot(m, a) * a)
b3 = cross(b2, b1)

% TRIAD rotation matrix
disp("Building TRIAD rotation matrix");
R = [
  b1, b2, b3
]

% Roll / pitch / yaw from rotation matrix
disp("Obtain Euler angles");

phi = atan2(R(3,2), R(3,3))
theta = asin(-R(3,1))
psi = atan2(R(2,1), R(1,1))
angles = [phi; theta; psi]

% Partial derivative of the matrix with respect to the accelerometer readings
disp("Jacobian of Euler angles with respect to accelerometer readings");
Ja = jacobian(angles, a);
Ja = simplify(Ja)

% disp("Measurement noise from accelerometer")
% R_measacc = simplify(Ja*R_acc*Ja')

disp("Measurement noise from accelerometer");
syms Ja11 Ja12 Ja13 Ja21 Ja22 Ja23 Ja31 Ja32 Ja33;
zero = sym(0);
assume([Ja11 Ja12 Ja13 Ja21 Ja22 Ja23 Ja31 Ja32 Ja33], 'real');
Jasimple = [Ja11, Ja12, Ja13; zero, zero, Ja23; Ja31, Ja32, Ja33]

R_measacc = simplify(Jasimple*R_acc*Jasimple')


% Partial derivative of the matrix with respect to the accelerometer readings
disp("Jacobian of Euler angles with respect to magnetometer readings");
Jm = jacobian(angles, m);
Jm = simplify(Jm)

disp("Measurement noise from magnetometer")
syms Jm11 Jm12 Jm13 Jm21 Jm22 Jm23 Jm31 Jm32 Jm33;
assume([Jm11 Jm12 Jm13 Jm21 Jm22 Jm23 Jm31 Jm32 Jm33], 'real');
Jmsimple = [Jm11, Jm12, Jm13; zero, zero, zero; zero, zero, zero]

R_measmag = simplify(Jmsimple*R_mag*Jmsimple')

% Full equation
disp("Full measurement noise (simplified)")
R_meas = R_measacc + R_measmag

disp("Full measurement noise")
R_meas = Ja*R_acc*Ja.' + Jm*R_mag*Jm.'

% disp("Introduce dot product of acc and mag")
syms dam;
assume([dam], 'real')

% Substitute dot product with dam
Ja_exp = expand(Ja);
Jm_exp = expand(Jm);

R_meas = Ja_exp*R_acc*Ja_exp.' + Jm_exp*R_mag*Jm_exp.';

% Substitute dot products with sum
% R_meas = subs(R_meas, a_x*m_x+a_y*m_y+a_z*m_z, dam);
% R_meas = subs(R_meas, 2*a_x*m_x+a_y*m_y+a_z*m_z, a_x*m_x + dam);
% R_meas = subs(R_meas, a_x*m_x+2*a_y*m_y+a_z*m_z, a_y*m_y + dam);
% R_meas = subs(R_meas, a_x*m_x+a_y*m_y+2*a_z*m_z, a_z*m_z + dam);

R_meas = simplify(R_meas)

% Substitute dot products with sum
% R_meas = subs(R_meas, a_x*m_x+a_y*m_y+a_z*m_z, dam);
% R_meas = subs(R_meas, 2*a_x*m_x+a_y*m_y+a_z*m_z, a_x*m_x + dam);
% R_meas = subs(R_meas, a_x*m_x+2*a_y*m_y+a_z*m_z, a_y*m_y + dam);
% R_meas = subs(R_meas, a_x*m_x+a_y*m_y+2*a_z*m_z, a_z*m_z + dam);

% Convert the symbolic matrix to LaTeX string
latex_str = latex(R_meas);

% Create LaTeX document
fid = fopen('triad.tex', 'w');
fprintf(fid, '\\documentclass{standalone}\n');
fprintf(fid, '\\usepackage{amsmath}\n');
fprintf(fid, '\\begin{document}\n');
fprintf(fid, '$\n');
fprintf(fid, 'R_{meas} = %s\n', latex_str);
fprintf(fid, '$\n');
fprintf(fid, '\\end{document}\n');
fclose(fid);

% Compile LaTeX document to PDF
system('pdflatex triad.tex');

% Convert PDF to PNG image
% system('convert -density 300 matrix.pdf -quality 90 matrix.png');
