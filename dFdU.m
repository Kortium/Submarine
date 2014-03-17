function dFdU = dFdU(dT,q1,q2,q3,q4,w1,w2,w3)
%DFDU
%    DFDU = DFDU(DT,Q1,Q2,Q3,Q4,W1,W2,W3)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    17-Mar-2014 15:37:47

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
t18 = conj(q4);
t19 = conj(q1);
t20 = conj(w3);
t21 = conj(q2);
t22 = conj(w2);
t23 = conj(q3);
t24 = conj(w1);
t26 = t17.*t23;
t27 = t13.*t14.*t15.*t19.*t22;
t28 = t13.*t14.*t15.*t20.*t21;
t29 = t13.*t14.*t15.*t18.*t24;
t16 = t26+t27-t28+t29;
t30 = t17.*t18;
t31 = t13.*t14.*t15.*t19.*t20;
t32 = t13.*t14.*t15.*t21.*t22;
t33 = t13.*t14.*t15.*t23.*t24;
t25 = t30+t31+t32-t33;
t34 = t17.*t21;
t35 = t13.*t14.*t15.*t19.*t24;
t36 = t13.*t14.*t15.*t20.*t23;
t46 = t13.*t14.*t15.*t18.*t22;
t37 = t34+t35+t36-t46;
t38 = q2.*t13.*t14.*t15.*t24;
t39 = q3.*t13.*t14.*t15.*t22;
t40 = q4.*t13.*t14.*t15.*t20;
t48 = t17.*t19;
t41 = t38+t39+t40-t48;
t42 = t16.^2;
t43 = t42.*2.0;
t44 = t25.^2;
t45 = t44.*2.0;
t47 = t16.*t37.*2.0;
t49 = t47-t25.*t41.*2.0;
t50 = t25.*t37.*2.0;
t51 = t16.*t41.*2.0;
t52 = t50+t51;
dFdU = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,-dT.*(t43+t45-1.0),dT.*t49,dT.*t52,0.0,0.0,0.0,0.0,-t43-t45+1.0,t49,t52,0.0,0.0,0.0],[13, 4]);
