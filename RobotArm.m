syms th2 th3 th4 th5 th6;
% [th2, th3, th5] = deal(19*pi/180, (50-45)*pi/180, 0);
% th4 = 11*pi/180 + th3;
% th6 = th3 - th4;
%th5 = 0
[a2, a3, a6] = deal(1.3, 12.021, 13);
[d1, d2, d5] = deal(6.1, 7.001, 12.171);
[k3, k4, k6] = deal(pi/4, -pi/4, pi/2);

% ----------------------- DH table -----------------------
DH_table = [     0,     0,     0,    d1;
               th2,  pi/2,    a2,    d2;
            th3+k3,    pi,    a3,     0;
            th4+k4, -pi/2,     0,     0;
               th5,  pi/2,     0,    d5;
            th6+k6,    pi,    a6,     0];

% ----------------------- Forward kinematics - finding homogeneous transformation matrices -----------------------
A1 = DH_HTM(DH_table(1,:), 'r');
A12 = DH_HTM(DH_table([1 2],:), 'r');
A123 = DH_HTM(DH_table([1 2 3],:), 'r');
A1234 = DH_HTM(DH_table([1 2 3 4],:), 'r');
A12345 = DH_HTM(DH_table([1 2 3 4 5],:), 'r');
A123456 = DH_HTM(DH_table([1 2 3 4 5 6],:), 'r');

H = DH_HTM(DH_table, 'r')

% ----------------------- Finding the Jacobian matrix -----------------------
z0 = [0 0 1].';
z1 = A1([1 2 3],3);
z2 = A12([1 2 3],3);
z3 = A123([1 2 3],3);
z4 = A1234([1 2 3],3);
z5 = A12345([1 2 3],3);
t0 = [0 0 0].';
t1 = A1([1 2 3],4);
t2 = A12([1 2 3],4);
t3 = A123([1 2 3],4);
t4 = A1234([1 2 3],4);
t5 = A12345([1 2 3],4);
t6 = A123456([1 2 3],4);

J1 = [cross(z0,(t6-t0));z0];
J2 = [cross(z1,(t6-t1));z1];
J3 = [cross(z2,(t6-t2));z2];
J4 = [cross(z3,(t6-t3));z3];
J5 = [cross(z4,(t6-t4));z4];
J6 = [cross(z5,(t6-t5));z5];
J = [J1 J2 J3 J4 J5 J6]

% ----------------------- Forward kinematics for the initial position of the robot -----------------------
L1=Link([1 d1+d2 a2 pi/2 0 0]);
L1(2)=Link([1 0 a3 pi 0 k3]);
L1(3)=Link([1 0 0 -pi/2 0 k4]);
L1(4)=Link([1 d5 0 pi/2 0 0]);
L1(5)=Link([1 0 a6 pi 0 k6]);

R=SerialLink(L1)
fwd = R.fkine([0,0,0,0,0])

% ----------------------- Verify the inverse kinematics -----------------------
[th2, th3, th5] = deal(19*pi/180, (50-45)*pi/180, 0);
th4 = 11*pi/180 + th3;
th6 = th3 - th4;
fwd = R.fkine([th2,th3,th4,th5,th6])
R.plot([th2,th3,th4,th5,th6])

function [DH_HTM] = DH_HTM(Matrix,angtype)
% Input Matrix: DH Table of (n,4) Dimension, else throw error
% Output matrix: Homogenous transformation: Dimension (4,4)
if size(Matrix,2) ~= 4
    error("Matrix must have 4 columns");
end
output = eye(4);
len = size(Matrix,1); % Number of Rows
for i = 1 : len
    params = Matrix(i,:);
    theta = params(1);
    alpha = params(2);
    rx = params(3);
    dz = params(4);
    next = dh_link(theta,alpha,rx,dz,angtype);
    output = output * next;
end
output = simplify(output);
DH_HTM = output;
end

function [matrix] = dh_link(theta,alpha,rx,dz,angle)
if angle == "d"
    matrix = [
        cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) rx*cosd(theta);...
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) rx*sind(theta);...
        0 sind(alpha) cosd(alpha) dz;...
        0 0 0 1 ...
    ];
end
if angle == "r"
    matrix = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) rx*cos(theta);...
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) rx*sin(theta);...
        0 sin(alpha) cos(alpha) dz;...
        0 0 0 1 ...
    ];
end
matrix = vpa(matrix);
end