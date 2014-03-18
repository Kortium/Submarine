function DCM = quat2dcm_cc(q)
q1 =q(1);
q2 =q(2);
q3 =q(3);
q4 =q(4);

%syms DCM

DCM(1,1) = 1 - 2*(q3^2 + q4^2);
DCM(1,2) = 2*(q2*q3 - q4*q1);
DCM(1,3) = 2*(q2*q4 + q3*q1);
DCM(2,1) = 2*(q2*q3 + q4*q1);
DCM(2,2) = 1 - 2*(q2^2 + q4^2);
DCM(2,3) = 2*(q3*q4 - q2*q1);
DCM(3,1) = 2*(q2*q4 - q3*q1);
DCM(3,2) = 2*(q3*q4 + q2*q1);
DCM(3,3) = 1 - 2*(q2^2 + q3^2);
end