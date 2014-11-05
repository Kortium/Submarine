function dGdZ = dGdZ(Phi,Rho,Theta,q1,q2,q3,q4)
%DGDZ
%    DGDZ = DGDZ(PHI,RHO,THETA,Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    06-Nov-2014 01:49:05

t2 = cos(Theta);
t3 = sin(Phi);
t4 = q1.*q4.*2.0;
t5 = q2.*q3.*2.0;
t6 = t4+t5;
t7 = cos(Phi);
t8 = sin(Theta);
t9 = q3.^2;
t10 = t9.*2.0;
t11 = q4.^2;
t12 = t11.*2.0;
t13 = t10+t12-1.0;
t14 = q1.*q3.*2.0;
t23 = q2.*q4.*2.0;
t15 = t14-t23;
t16 = t4-t5;
t17 = q2.^2;
t18 = t17.*2.0;
t19 = t12+t18-1.0;
t20 = q1.*q2.*2.0;
t21 = q3.*q4.*2.0;
t22 = t20+t21;
t24 = t14+t23;
t25 = t20-t21;
t26 = t10+t18-1.0;
dGdZ = reshape([-Rho.*t2.*t6.*t7-Rho.*t2.*t3.*t13,-Rho.*t2.*t3.*t16+Rho.*t2.*t7.*t19,Rho.*t2.*t3.*t24+Rho.*t2.*t7.*t25,Rho.*t2.*t15+Rho.*t3.*t6.*t8-Rho.*t7.*t8.*t13,-Rho.*t2.*t22-Rho.*t3.*t8.*t19-Rho.*t7.*t8.*t16,Rho.*t2.*t26-Rho.*t3.*t8.*t25+Rho.*t7.*t8.*t24,t8.*t15-t2.*t3.*t6+t2.*t7.*t13,-t8.*t22+t2.*t3.*t19+t2.*t7.*t16,t8.*t26+t2.*t3.*t25-t2.*t7.*t24],[3, 3]);
