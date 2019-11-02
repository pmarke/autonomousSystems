
P = sym('P',[9,9],'real');


% 3 landmarks
l = 1;
F = zeros(5,9);
F(1:3,1:3) = eye(3);
F(4:5,2+2*l:3+2*l) = eye(2);

H = sym('H',[2,5],'real');
Ht = H*F;

simplify(Ht*P*Ht')