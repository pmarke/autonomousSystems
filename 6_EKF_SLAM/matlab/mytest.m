
syms g1 g2 'real';
P = sym('P',[9,9],'real');
R = sym('R',[3,3],'real');

% 3 landmarks
l = 1;
F = zeros(3,9);
F(1:3,1:3) = eye(3);
% F(4:5,2+2*l:3+2*l) = eye(2);
gt = [ 0 0 g1; 0 0 g2; 0 0 0];
G = eye(9) + F'*gt*F;


G*P*G'+F'*R*F