function dFdU = dFdU(dT,q1,q2,q3,q4,w1,w2,w3)
%DFDU
%    DFDU = DFDU(DT,Q1,Q2,Q3,Q4,W1,W2,W3)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    21-Mar-2014 12:15:35

t2 = dT.^2;
t3 = w1.^2;
t4 = t2.*t3;
t5 = w2.^2;
t6 = t2.*t5;
t7 = w3.^2;
t8 = t2.*t7;
t9 = t4+t6+t8;
t10 = sqrt(t9);
t11 = conj(t10);
t12 = t11.*(1.0./2.0);
t13 = 1.0./t11;
t14 = conj(dT);
t15 = sin(t12);
t17 = cos(t12);
t18 = conj(q1);
t19 = conj(w1);
t20 = conj(w3);
t21 = conj(w2);
t33 = t17.*t18;
t34 = q2.*t13.*t14.*t15.*t19;
t35 = q3.*t13.*t14.*t15.*t21;
t36 = q4.*t13.*t14.*t15.*t20;
t16 = -t33+t34+t35+t36;
t23 = conj(q3);
t24 = conj(q2);
t25 = conj(q4);
t38 = t17.*t24;
t39 = t13.*t14.*t15.*t18.*t19;
t40 = t13.*t14.*t15.*t20.*t23;
t41 = t13.*t14.*t15.*t21.*t25;
t22 = t38+t39+t40-t41;
t28 = t17.*t23;
t29 = t13.*t14.*t15.*t18.*t21;
t30 = t13.*t14.*t15.*t20.*t24;
t31 = t13.*t14.*t15.*t19.*t25;
t26 = t28+t29-t30+t31;
t43 = t17.*t25;
t44 = t13.*t14.*t15.*t18.*t20;
t45 = t13.*t14.*t15.*t21.*t24;
t46 = t13.*t14.*t15.*t19.*t23;
t27 = t43+t44+t45-t46;
t32 = t26.^2;
t37 = t16.^2;
t42 = t22.^2;
t47 = t27.^2;
t48 = t32+t37+t42+t47;
t49 = 1.0./t48;
t50 = t32.*t49.*2.0;
t51 = t47.*t49.*2.0;
t52 = t16.*t27.*t49.*2.0;
t53 = t16.*t26.*t49.*2.0;
t54 = t22.*t27.*t49.*2.0;
t55 = t53+t54;
dFdU = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,-dT.*(t50+t51-1.0),-dT.*(t52-t22.*t26.*t49.*2.0),dT.*t55,0.0,0.0,0.0,0.0,-t50-t51+1.0,-t52+t22.*t26.*t49.*2.0,t55,0.0,0.0,0.0],[13, 4]);
