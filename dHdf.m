function dHdf = dHdf(q1,q2,q3,q4,x,xf,y,yf,z,zf)
%DHDF
%    DHDF = DHDF(Q1,Q2,Q3,Q4,X,XF,Y,YF,Z,ZF)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    06-Apr-2014 18:55:37

t2 = q4.^2;
t3 = q3.^2;
t4 = t2.*2.0;
t5 = x-xf;
t6 = y-yf;
t7 = q1.*q4.*2.0;
t8 = q2.*q3.*2.0;
t9 = z-zf;
t10 = q1.*q4;
t11 = q2.*q3;
t12 = t3.*2.0;
t13 = t4+t12-1.0;
t14 = t5.*t13;
t15 = q2.^2;
t16 = t15.*2.0;
t17 = t4+t16-1.0;
t18 = t6.*t17;
t19 = t7-t8;
t20 = t6.*t19;
t21 = q1.*q3.*2.0;
t22 = q2.*q4.*2.0;
t23 = t21+t22;
t24 = t9.*t23;
t25 = t7+t8;
t26 = t5.*t25;
t27 = q1.*q2.*2.0;
t33 = q3.*q4.*2.0;
t28 = t27-t33;
t29 = t9.*t28;
t30 = imag(t18);
t31 = real(t14);
t32 = imag(t26);
t34 = imag(t29);
t35 = real(t20);
t36 = real(t24);
t37 = t30-t31-t32+t34-t35+t36;
t38 = imag(t14);
t39 = real(t18);
t40 = imag(t20);
t41 = imag(t24);
t42 = real(t26);
t43 = real(t29);
t44 = t38+t39+t40-t41-t42+t43;
t45 = t37.^2;
t46 = real(t2);
t47 = t46.*2.0;
t48 = imag(t10);
t49 = t48.*2.0;
t50 = imag(t11);
t51 = t50.*2.0;
t52 = 1.0./t37;
t53 = imag(t2);
t54 = t53.*2.0;
t55 = real(t10);
t56 = real(t11);
t57 = 1.0./t37.^2;
t58 = t44.^2;
t59 = t45+t58;
t60 = 1.0./t59;
t61 = q1.*q2;
t62 = q3.*q4;
t63 = q1.*q3;
t64 = q2.*q4;
t65 = t18-t26+t29;
t66 = t14+t20-t24;
t68 = t12+t16-1.0;
t69 = t9.*t68;
t70 = t21-t22;
t71 = t5.*t70;
t72 = t27+t33;
t73 = t6.*t72;
t74 = t65.^2;
t75 = t66.^2;
t76 = t74+t75;
t77 = sqrt(t76);
t95 = real(t69);
t96 = real(t71);
t97 = real(t73);
t98 = imag(t77);
t67 = t95+t96-t97-t98;
t83 = imag(t69);
t84 = imag(t71);
t85 = imag(t73);
t86 = real(t77);
t78 = t83+t84-t85+t86;
t79 = real(t63);
t80 = t79.*2.0;
t81 = real(t64);
t82 = t81.*2.0;
t87 = 1.0./sqrt(t76);
t88 = t13.*t66.*2.0;
t125 = t25.*t65.*2.0;
t89 = t88-t125;
t90 = t87.*t89;
t91 = imag(t63);
t92 = t91.*2.0;
t93 = imag(t64);
t94 = t93.*2.0;
t99 = t78.^2;
t100 = t67.^2;
t101 = t99+t100;
t102 = 1.0./t101;
t103 = real(t61);
t104 = real(t62);
t105 = t104.*2.0;
t106 = 1.0./t78;
t107 = t17.*t65.*2.0;
t108 = t19.*t66.*2.0;
t109 = t107+t108;
t110 = t87.*t109;
t111 = imag(t61);
t112 = t111.*2.0;
t113 = imag(t62);
t114 = 1.0./t78.^2;
t115 = real(t15);
t116 = t115.*2.0;
t117 = real(t3);
t118 = t117.*2.0;
t119 = imag(t15);
t120 = t119.*2.0;
t121 = imag(t3);
t122 = t121.*2.0;
t123 = t23.*t66.*2.0;
t130 = t28.*t65.*2.0;
t124 = t123-t130;
t126 = t69+t71-t73;
t127 = t126.^2;
t128 = t74+t75+t127;
t129 = 1.0./sqrt(t128);
dHdf = reshape([t45.*t60.*(t52.*(t54-t55.*2.0-t56.*2.0+t122)+t44.*t57.*(t47+t49+t51+t118-1.0)),-t99.*t102.*(t106.*(-t80+t82+imag(t90).*(1.0./2.0))+t67.*t114.*(t92-t94+real(t90).*(1.0./2.0))),t129.*(t88-t125+t70.*t126.*2.0).*(-1.0./2.0),t45.*t60.*(t52.*(t47+t49-t51+t116-1.0)-t44.*t57.*(t54-t55.*2.0+t56.*2.0+t120)),-t99.*t102.*(t106.*(t103.*2.0+t105+imag(t110).*(1.0./2.0))-t67.*t114.*(t112+t113.*2.0-real(t110).*(1.0./2.0))),t129.*(t107+t108-t72.*t126.*2.0).*(-1.0./2.0),-t45.*t60.*(t52.*(t92+t94-t103.*2.0+t105)+t44.*t57.*(t80+t82+t112-t113.*2.0)),t99.*t102.*(t106.*(t116+t118+imag(t87.*t124).*(1.0./2.0)-1.0)-t67.*t114.*(t120+t122-real(t87.*t124).*(1.0./2.0))),t129.*(-t123+t130+t68.*t126.*2.0).*(-1.0./2.0)],[3, 3]);
