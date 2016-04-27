function [o] = CalculateEndEffectorLocation(q)
   % Kinematics for the Cyton Robot
   % Computed statically from getDHParams
   d = [37.9300 -4.6200 145.0000 11.0000 175.0000 7.4000 -7.6500 0 0];
   a = [0 0 0 0 0 67.7000 53.1500 8.0000 8.0000];
   % [A] matrices represent the kinematics of each joint
   piOver2 = pi/2;
   A1 = DH(a(1),  piOver2, d(1), q(1) );
   A2 = DH(a(2), -piOver2, d(2), q(2) );
   A3 = DH(a(3),  piOver2, d(3), q(3) );
   A4 = DH(a(4), -piOver2, d(4), q(4) );
   A5 = DH(a(5),  piOver2, d(5), q(5) );
   A6 = DH(a(6),  piOver2, d(6), q(6) + piOver2 );
   A7 = DH(a(7),  0, d(7), q(7) );
   % [T] matrices represent the transform from the base to each joint
   T_0_7 = A1*A2*A3*A4*A5*A6*A7;
   
   o = T_0_7(1:end-1,4)';
   
end
