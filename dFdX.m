function dFdX = dFdX(dT,q1,q2,q3,q4,w1,w2,w3)
%DFDX
%    DFDX = DFDX(DT,Q1,Q2,Q3,Q4,W1,W2,W3)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    06-Apr-2014 18:54:43

t2 = dT.^2;
t3 = w1.^2;
t4 = t2.*t3;
t5 = w2.^2;
t6 = t2.*t5;
t7 = w3.^2;
t8 = t2.*t7;
t9 = t4+t6+t8;
t10 = sqrt(t9);
t11 = t10.*(1.0./2.0);
t12 = cos(t11);
t13 = sin(t11);
t14 = 1.0./sqrt(t9);
t26 = q2.*t12;
t27 = dT.*q1.*t13.*t14.*w1;
t28 = dT.*q3.*t13.*t14.*w3;
t29 = dT.*q4.*t13.*t14.*w2;
t15 = t26+t27+t28-t29;
t30 = q3.*t12;
t31 = dT.*q1.*t13.*t14.*w2;
t32 = dT.*q2.*t13.*t14.*w3;
t33 = dT.*q4.*t13.*t14.*w1;
t16 = t30+t31-t32+t33;
t34 = q4.*t12;
t35 = dT.*q1.*t13.*t14.*w3;
t36 = dT.*q2.*t13.*t14.*w2;
t37 = dT.*q3.*t13.*t14.*w1;
t17 = t34+t35+t36-t37;
t19 = q1.*t12;
t20 = conj(q2);
t21 = dT.*t13.*t14.*t20.*w1;
t22 = conj(q3);
t23 = dT.*t13.*t14.*t22.*w2;
t24 = conj(q4);
t25 = dT.*t13.*t14.*t24.*w3;
t18 = -t19+t21+t23+t25;
t38 = t15.^2;
t39 = t16.^2;
t40 = t17.^2;
t41 = t18.^2;
t42 = t38+t39+t40+t41;
t43 = 1.0./t42.^(3.0./2.0);
t44 = 1.0./sqrt(t42);
t45 = 1.0./t9;
t46 = 1.0./t9.^(3.0./2.0);
t47 = dT.*t13.*t14.*t20;
t48 = q1.*t2.*t13.*t14.*w1.*(1.0./2.0);
t49 = dT.*t2.*t3.*t12.*t20.*t45.*(1.0./2.0);
t50 = dT.*t2.*t12.*t22.*t45.*w1.*w2.*(1.0./2.0);
t51 = dT.*t2.*t12.*t24.*t45.*w1.*w3.*(1.0./2.0);
t95 = dT.*t2.*t3.*t13.*t20.*t46;
t96 = dT.*t2.*t13.*t22.*t46.*w1.*w2;
t97 = dT.*t2.*t13.*t24.*t46.*w1.*w3;
t52 = t47+t48+t49+t50+t51-t95-t96-t97;
t53 = dT.*t13.*t14.*t22;
t54 = q1.*t2.*t13.*t14.*w2.*(1.0./2.0);
t55 = dT.*t2.*t5.*t12.*t22.*t45.*(1.0./2.0);
t56 = dT.*t2.*t12.*t20.*t45.*w1.*w2.*(1.0./2.0);
t57 = dT.*t2.*t12.*t24.*t45.*w2.*w3.*(1.0./2.0);
t111 = dT.*t2.*t5.*t13.*t22.*t46;
t112 = dT.*t2.*t13.*t20.*t46.*w1.*w2;
t113 = dT.*t2.*t13.*t24.*t46.*w2.*w3;
t58 = t53+t54+t55+t56+t57-t111-t112-t113;
t59 = dT.*q1.*t13.*t14;
t60 = dT.*q4.*t2.*t13.*t46.*w1.*w2;
t61 = dT.*q4.*t13.*t14;
t62 = dT.*q1.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0);
t63 = dT.*t13.*t14.*t24;
t64 = q1.*t2.*t13.*t14.*w3.*(1.0./2.0);
t65 = dT.*t2.*t7.*t12.*t24.*t45.*(1.0./2.0);
t66 = dT.*t2.*t12.*t20.*t45.*w1.*w3.*(1.0./2.0);
t67 = dT.*t2.*t12.*t22.*t45.*w2.*w3.*(1.0./2.0);
t125 = dT.*t2.*t7.*t13.*t24.*t46;
t126 = dT.*t2.*t13.*t20.*t46.*w1.*w3;
t127 = dT.*t2.*t13.*t22.*t46.*w2.*w3;
t68 = t63+t64+t65+t66+t67-t125-t126-t127;
t69 = dT.*q3.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0);
t70 = dT.*q2.*t2.*t13.*t46.*w2.*w3;
t71 = dT.*q2.*t13.*t14;
t72 = dT.*q1.*t2.*t12.*t45.*w2.*w3.*(1.0./2.0);
t73 = dT.*q3.*t13.*t14;
t74 = dT.*q1.*t2.*t13.*t46.*w1.*w3;
t75 = dT.*t13.*t14.*t15.*w1.*2.0;
t76 = dT.*t13.*t14.*t16.*w2.*2.0;
t77 = dT.*t13.*t14.*t17.*w3.*2.0;
t141 = t12.*t18.*2.0;
t78 = t75+t76+t77-t141;
t79 = t12.*t44;
t80 = t12.*t15.*2.0;
t81 = dT.*t13.*t14.*t17.*w2.*2.0;
t82 = dT.*t13.*t14.*t18.*w1.*2.0;
t142 = dT.*t13.*t14.*t16.*w3.*2.0;
t83 = t80+t81+t82-t142;
t84 = t12.*t16.*2.0;
t85 = dT.*t13.*t14.*t15.*w3.*2.0;
t86 = dT.*t13.*t14.*t18.*w2.*2.0;
t144 = dT.*t13.*t14.*t17.*w1.*2.0;
t87 = t84+t85+t86-t144;
t88 = t12.*t17.*2.0;
t89 = dT.*t13.*t14.*t16.*w1.*2.0;
t90 = dT.*t13.*t14.*t18.*w3.*2.0;
t145 = dT.*t13.*t14.*t15.*w2.*2.0;
t91 = t88+t89+t90-t145;
t92 = dT.*q1.*t2.*t3.*t12.*t45.*(1.0./2.0);
t93 = dT.*q4.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0);
t94 = dT.*q3.*t2.*t13.*t46.*w1.*w3;
t98 = t18.*t52.*2.0;
t99 = q4.*t2.*t13.*t14.*w1.*(1.0./2.0);
t100 = dT.*q3.*t2.*t3.*t12.*t45.*(1.0./2.0);
t101 = dT.*q1.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0);
t102 = dT.*q2.*t2.*t13.*t46.*w1.*w2;
t149 = q2.*t2.*t13.*t14.*w1.*(1.0./2.0);
t150 = dT.*q1.*t2.*t3.*t13.*t46;
t103 = t59+t60+t69+t92-t93-t94-t149-t150;
t104 = dT.*q4.*t2.*t3.*t12.*t45.*(1.0./2.0);
t105 = dT.*q1.*t2.*t13.*t46.*w1.*w2;
t106 = dT.*q2.*t2.*t13.*t46.*w1.*w3;
t107 = q2.*t2.*t13.*t14.*w2.*(1.0./2.0);
t108 = dT.*q4.*t2.*t5.*t12.*t45.*(1.0./2.0);
t109 = dT.*q3.*t2.*t13.*t46.*w2.*w3;
t120 = dT.*q4.*t2.*t5.*t13.*t46;
t121 = dT.*q3.*t2.*t12.*t45.*w2.*w3.*(1.0./2.0);
t110 = t61-t62+t105+t107+t108+t109-t120-t121;
t114 = t18.*t58.*2.0;
t115 = dT.*q1.*t2.*t5.*t12.*t45.*(1.0./2.0);
t116 = dT.*q2.*t2.*t12.*t45.*w2.*w3.*(1.0./2.0);
t117 = dT.*q2.*t2.*t5.*t12.*t45.*(1.0./2.0);
t118 = dT.*q1.*t2.*t13.*t46.*w2.*w3;
t119 = dT.*q3.*t2.*t13.*t46.*w1.*w2;
t122 = dT.*q3.*t2.*t7.*t12.*t45.*(1.0./2.0);
t123 = dT.*q4.*t2.*t13.*t46.*w2.*w3;
t136 = q2.*t2.*t13.*t14.*w3.*(1.0./2.0);
t137 = dT.*q3.*t2.*t7.*t13.*t46;
t138 = dT.*q4.*t2.*t12.*t45.*w2.*w3.*(1.0./2.0);
t124 = t73-t74+t101+t122+t123-t136-t137-t138;
t128 = t18.*t68.*2.0;
t129 = dT.*q1.*t2.*t7.*t12.*t45.*(1.0./2.0);
t166 = q4.*t2.*t13.*t14.*w3.*(1.0./2.0);
t167 = dT.*q1.*t2.*t7.*t13.*t46;
t130 = t59-t69-t70+t94+t116+t129-t166-t167;
t131 = t17.*t130.*2.0;
t132 = q3.*t2.*t13.*t14.*w3.*(1.0./2.0);
t133 = dT.*q2.*t2.*t7.*t12.*t45.*(1.0./2.0);
t134 = dT.*q4.*t2.*t13.*t46.*w1.*w3;
t164 = dT.*q2.*t2.*t7.*t13.*t46;
t165 = dT.*q4.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0);
t135 = t71-t72+t118+t132+t133+t134-t164-t165;
t139 = t15.*t124.*2.0;
t168 = t16.*t135.*2.0;
t140 = t128+t131+t139-t168;
t143 = dT.*t13.*t14.*t44.*w3;
t146 = dT.*t13.*t14.*t44.*w1;
t152 = q3.*t2.*t13.*t14.*w1.*(1.0./2.0);
t153 = dT.*q4.*t2.*t3.*t13.*t46;
t154 = dT.*q2.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0);
t147 = t61+t62+t104-t105+t106-t152-t153-t154;
t170 = dT.*q3.*t2.*t3.*t13.*t46;
t171 = dT.*q2.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0);
t148 = t73+t74+t99+t100-t101+t102-t170-t171;
t151 = t15.*t103.*2.0;
t155 = t16.*t147.*2.0;
t172 = t17.*t148.*2.0;
t156 = t98+t151+t155-t172;
t158 = q3.*t2.*t13.*t14.*w2.*(1.0./2.0);
t159 = dT.*q1.*t2.*t5.*t13.*t46;
t157 = t59-t60+t70+t93+t115-t116-t158-t159;
t160 = t16.*t157.*2.0;
t173 = q4.*t2.*t13.*t14.*w2.*(1.0./2.0);
t174 = dT.*q2.*t2.*t5.*t13.*t46;
t175 = dT.*q3.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0);
t161 = t71+t72+t117-t118+t119-t173-t174-t175;
t162 = t17.*t161.*2.0;
t176 = t15.*t110.*2.0;
t163 = t114+t160+t162-t176;
t169 = dT.*t13.*t14.*t44.*w2;
dFdX = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t79+t18.*t43.*t78.*(1.0./2.0),t146-t15.*t43.*t78.*(1.0./2.0),t169-t16.*t43.*t78.*(1.0./2.0),t143-t17.*t43.*t78.*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18.*t43.*t83.*(1.0./2.0)-dT.*t13.*t14.*t44.*w1,t79-t15.*t43.*t83.*(1.0./2.0),-t143-t16.*t43.*t83.*(1.0./2.0),t169-t17.*t43.*t83.*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18.*t43.*t87.*(1.0./2.0)-dT.*t13.*t14.*t44.*w2,t143-t15.*t43.*t87.*(1.0./2.0),t79-t16.*t43.*t87.*(1.0./2.0),-t146-t17.*t43.*t87.*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t18.*t43.*t91.*(1.0./2.0)-dT.*t13.*t14.*t44.*w3,t15.*t43.*t91.*(-1.0./2.0)-dT.*t13.*t14.*t44.*w2,t146-t16.*t43.*t91.*(1.0./2.0),t79-t17.*t43.*t91.*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,dT,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,dT,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,dT,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-t44.*t52+t18.*t43.*(t98-t17.*(t73+t74+t99+t100+t102-dT.*q3.*t2.*t3.*t13.*t46-dT.*q1.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0)-dT.*q2.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0)).*2.0+t15.*(t59+t60+t69+t92-q2.*t2.*t13.*t14.*w1.*(1.0./2.0)-dT.*q1.*t2.*t3.*t13.*t46-dT.*q4.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0)-dT.*q3.*t2.*t13.*t46.*w1.*w3).*2.0+t16.*(t61+t62+t104+t106-q3.*t2.*t13.*t14.*w1.*(1.0./2.0)-dT.*q4.*t2.*t3.*t13.*t46-dT.*q1.*t2.*t13.*t46.*w1.*w2-dT.*q2.*t2.*t12.*t45.*w1.*w3.*(1.0./2.0)).*2.0).*(1.0./2.0),t44.*t103-t15.*t43.*t156.*(1.0./2.0),t44.*t147-t16.*t43.*t156.*(1.0./2.0),-t44.*t148-t17.*t43.*t156.*(1.0./2.0),0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,-t44.*t58+t18.*t43.*(t114-t15.*t110.*2.0+t17.*(t71+t72+t117+t119-q4.*t2.*t13.*t14.*w2.*(1.0./2.0)-dT.*q2.*t2.*t5.*t13.*t46-dT.*q3.*t2.*t12.*t45.*w1.*w2.*(1.0./2.0)-dT.*q1.*t2.*t13.*t46.*w2.*w3).*2.0+t16.*(t59-t60+t70+t93+t115-q3.*t2.*t13.*t14.*w2.*(1.0./2.0)-dT.*q1.*t2.*t5.*t13.*t46-dT.*q2.*t2.*t12.*t45.*w2.*w3.*(1.0./2.0)).*2.0).*(1.0./2.0),-t44.*t110-t15.*t43.*t163.*(1.0./2.0),t44.*t157-t16.*t43.*t163.*(1.0./2.0),t44.*t161-t17.*t43.*t163.*(1.0./2.0),0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,-t44.*t68+t18.*t43.*t140.*(1.0./2.0),t44.*t124-t15.*t43.*t140.*(1.0./2.0),-t44.*t135-t16.*t43.*t140.*(1.0./2.0),t44.*t130-t17.*t43.*t140.*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,1.0],[13, 13]);