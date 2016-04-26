function J = numericJacobian(q)
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
   T_0_1 = A1;
   T_0_2 = A1*A2;
   T_0_3 = A1*A2*A3;
   T_0_4 = A1*A2*A3*A4;
   T_0_5 = A1*A2*A3*A4*A5;
   T_0_6 = A1*A2*A3*A4*A5*A6;
   T_0_7 = A1*A2*A3*A4*A5*A6*A7;
   % Define components for Jacobian:
   % z-axis is just the 3rd column of the transforms
   z0 = [0 0 1]';
   z1 = T_0_1(1:3,3);
   z2 = T_0_2(1:3,3);
   z3 = T_0_3(1:3,3);
   z4 = T_0_4(1:3,3);
   z5 = T_0_5(1:3,3);
   z6 = T_0_6(1:3,3);
   % The joint centers are just the 4th column from each transformation
   % matrix to the base
   o0 = [0 0 0]';
   o1 = T_0_1(1:3,4,1);
   o2 = T_0_2(1:3,4,1);
   o3 = T_0_3(1:3,4,1);
   o4 = T_0_4(1:3,4,1);
   o5 = T_0_5(1:3,4,1);
   o6 = T_0_6(1:3,4,1);
   o7 = T_0_7(1:3,4,1);
   % Per Eq. 4.64, J is the Geometric Jacobian
   oc = o7;
   J1 = cross(z0,(oc-o0));
   J2 = cross(z1,(oc-o1));
   J3 = cross(z2,(oc-o2));
   J4 = cross(z3,(oc-o3));
   J5 = cross(z4,(oc-o4));
   J6 = cross(z5,(oc-o5));
   J7 = cross(z6,(oc-o6));
   J11 = [J1 J2 J3 J4 J5 J6 J7];
   
   J = [J11; z0 z1 z2 z3 z4 z5 z6];
   
end