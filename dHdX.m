function dHdX = dHdX(q1,q2,q3,q4,x,xf,y,yf,z,zf)
%DHDX
%    DHDX = DHDX(Q1,Q2,Q3,Q4,X,XF,Y,YF,Z,ZF)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    26-Feb-2014 16:09:13

t2 = q1.*q4.*2.0;
t3 = q2.*q3.*2.0;
t4 = z-zf;
t5 = y-yf;
t6 = q1.^2;
t7 = q2.^2;
t8 = q3.^2;
t9 = q4.^2;
t10 = x-xf;
t11 = q1.*q4;
t12 = q2.*q3;
t13 = t2-t3;
t14 = t5.*t13;
t15 = q1.*q3.*2.0;
t16 = q2.*q4.*2.0;
t17 = t15+t16;
t18 = t4.*t17;
t19 = t2+t3;
t20 = t10.*t19;
t21 = q1.*q2.*2.0;
t29 = q3.*q4.*2.0;
t22 = t21-t29;
t23 = t4.*t22;
t24 = t6+t7-t8-t9;
t25 = t10.*t24;
t26 = t6-t7+t8-t9;
t27 = t5.*t26;
t28 = imag(t20);
t30 = imag(t23);
t31 = real(t14);
t32 = real(t18);
t33 = imag(t27);
t34 = real(t25);
t35 = t28-t30+t31-t32+t33-t34;
t36 = imag(t14);
t37 = imag(t18);
t38 = real(t20);
t39 = real(t23);
t40 = imag(t25);
t41 = real(t27);
t42 = -t36+t37+t38-t39+t40+t41;
t43 = t35.^2;
t44 = real(t6);
t45 = real(t7);
t46 = real(t8);
t47 = real(t9);
t48 = imag(t11);
t49 = t48.*2.0;
t50 = imag(t12);
t51 = t50.*2.0;
t52 = 1.0./t35;
t53 = imag(t6);
t54 = imag(t7);
t55 = imag(t8);
t56 = imag(t9);
t57 = real(t11);
t58 = t57.*2.0;
t59 = real(t12);
t60 = t59.*2.0;
t61 = 1.0./t35.^2;
t62 = t42.^2;
t63 = t43+t62;
t64 = 1.0./t63;
t65 = q1.*q2;
t66 = q3.*q4;
t67 = q1.*q3;
t68 = q2.*q4;
t69 = q4.*t10;
t70 = q1.*t5;
t71 = q2.*t4;
t72 = q1.*t10;
t73 = q4.*t5;
t74 = q3.*t4;
t75 = q3.*t10;
t76 = q2.*t5;
t77 = q1.*t4;
t78 = q2.*t10;
t79 = q3.*t5;
t80 = q4.*t4;
t81 = imag(t75);
t82 = imag(t76);
t83 = t82.*2.0;
t84 = imag(t77);
t85 = t84.*2.0;
t86 = real(t78);
t87 = t86.*2.0;
t88 = real(t79);
t89 = t88.*2.0;
t90 = real(t80);
t91 = t90.*2.0;
t178 = t81.*2.0;
t92 = t83+t85+t87+t89+t91-t178;
t93 = imag(t78);
t94 = t93.*2.0;
t95 = imag(t79);
t96 = t95.*2.0;
t97 = imag(t80);
t98 = t97.*2.0;
t99 = real(t75);
t100 = t99.*2.0;
t101 = real(t76);
t102 = real(t77);
t176 = t101.*2.0;
t177 = t102.*2.0;
t103 = t94+t96+t98+t100-t176-t177;
t104 = imag(t69);
t105 = t104.*2.0;
t106 = imag(t70);
t107 = t106.*2.0;
t108 = imag(t71);
t109 = real(t72);
t110 = real(t73);
t111 = t110.*2.0;
t112 = real(t74);
t190 = t108.*2.0;
t200 = t109.*2.0;
t201 = t112.*2.0;
t113 = t105+t107+t111-t190-t200-t201;
t114 = imag(t72);
t115 = t114.*2.0;
t116 = imag(t73);
t117 = imag(t74);
t118 = t117.*2.0;
t119 = real(t69);
t120 = t119.*2.0;
t121 = real(t70);
t122 = t121.*2.0;
t123 = real(t71);
t189 = t123.*2.0;
t203 = t116.*2.0;
t124 = t115+t118+t120+t122-t189-t203;
t125 = -t14+t18+t25;
t126 = t20-t23+t27;
t127 = real(t67);
t128 = t127.*2.0;
t129 = real(t68);
t130 = t129.*2.0;
t131 = t125.^2;
t132 = t126.^2;
t133 = t131+t132;
t134 = 1.0./sqrt(t133);
t135 = t24.*t125.*2.0;
t136 = t19.*t126.*2.0;
t137 = t135+t136;
t138 = t134.*t137;
t139 = imag(t67);
t140 = t139.*2.0;
t141 = imag(t68);
t142 = t141.*2.0;
t143 = t15-t16;
t144 = t10.*t143;
t145 = imag(t144);
t146 = t21+t29;
t147 = t5.*t146;
t148 = imag(t147);
t149 = sqrt(t133);
t150 = real(t149);
t151 = t6-t7-t8+t9;
t152 = t4.*t151;
t153 = imag(t152);
t154 = -t145+t148+t150+t153;
t155 = imag(t149);
t156 = real(t144);
t157 = real(t147);
t158 = real(t152);
t159 = t155+t156-t157-t158;
t160 = t154.^2;
t161 = real(t65);
t162 = real(t66);
t163 = t162.*2.0;
t164 = 1.0./t154;
t165 = t13.*t125.*2.0;
t217 = t26.*t126.*2.0;
t166 = t165-t217;
t167 = imag(t65);
t168 = t167.*2.0;
t169 = imag(t66);
t170 = 1.0./t154.^2;
t171 = t159.^2;
t172 = t160+t171;
t173 = 1.0./t172;
t174 = t17.*t125.*2.0;
t218 = t22.*t126.*2.0;
t175 = t174-t218;
t179 = q4.*t10.*2.0;
t180 = q1.*t5.*2.0;
t208 = q2.*t4.*2.0;
t181 = t179+t180-t208;
t182 = t126.*t181.*2.0;
t183 = q1.*t10.*2.0;
t184 = q3.*t4.*2.0;
t209 = q4.*t5.*2.0;
t185 = t183+t184-t209;
t186 = t125.*t185.*2.0;
t187 = t182+t186;
t188 = t134.*t187;
t191 = q2.*t5.*2.0;
t192 = q1.*t4.*2.0;
t202 = q3.*t10.*2.0;
t193 = t191+t192-t202;
t194 = q2.*t10.*2.0;
t195 = q3.*t5.*2.0;
t196 = q4.*t4.*2.0;
t197 = t194+t195+t196;
t219 = t126.*t193.*2.0;
t220 = t125.*t197.*2.0;
t198 = t219-t220;
t199 = t134.*t198;
t204 = t125.*t193.*2.0;
t205 = t126.*t197.*2.0;
t206 = t204+t205;
t207 = t134.*t206;
t210 = t125.*t181.*2.0;
t221 = t126.*t185.*2.0;
t211 = t210-t221;
t212 = t134.*t211;
t213 = -t144+t147+t152;
t214 = t213.^2;
t215 = t131+t132+t214;
t216 = 1.0./sqrt(t215);
dHdX = reshape([-t43.*t64.*(t52.*(t53+t54-t55-t56+t58+t60)-t42.*t61.*(-t44-t45+t46+t47+t49+t51)),t160.*t173.*(t164.*(t128-t130+imag(t138).*(1.0./2.0))-t159.*t170.*(-t140+t142+real(t138).*(1.0./2.0))),t216.*(t135+t136-t143.*t213.*2.0).*(1.0./2.0),-t43.*t64.*(t52.*(t44-t45+t46-t47-t49+t51)-t42.*t61.*(t53-t54+t55-t56+t58-t60)),-t160.*t173.*(t164.*(t161.*2.0+t163+imag(t134.*t166).*(1.0./2.0))+t159.*t170.*(t168+t169.*2.0-real(t134.*t166).*(1.0./2.0))),t216.*(-t165+t217+t146.*t213.*2.0).*(1.0./2.0),-t43.*t64.*(t52.*(t140+t142-t161.*2.0+t163)+t42.*t61.*(t128+t130+t168-t169.*2.0)),t160.*t173.*(t164.*(-t44+t45+t46-t47+imag(t134.*t175).*(1.0./2.0))-t159.*t170.*(t53-t54-t55+t56+real(t134.*t175).*(1.0./2.0))),t216.*(t174-t218+t151.*t213.*2.0).*(1.0./2.0),-t43.*t64.*(t52.*t124-t42.*t61.*t113),t160.*t173.*(t164.*(t100-t176-t177+imag(t188).*(1.0./2.0))-t159.*t170.*(t83+t85-t178+real(t188).*(1.0./2.0))),t216.*(t182+t186+t193.*t213.*2.0).*(1.0./2.0),-t43.*t64.*(t52.*t103+t42.*t61.*t92),-t160.*t173.*(t164.*(t120+t122-t189+imag(t199).*(1.0./2.0))+t159.*t170.*(t105+t107-t190-real(t199).*(1.0./2.0))),t216.*(-t219+t220+t181.*t213.*2.0).*(1.0./2.0),-t43.*t64.*(t52.*t92-t42.*t61.*t103),t160.*t173.*(t164.*(-t111+t200+t201+imag(t207).*(1.0./2.0))+t159.*t170.*(t115+t118-t203-real(t207).*(1.0./2.0))),t216.*(t204+t205-t185.*t213.*2.0).*(1.0./2.0),t43.*t64.*(t52.*t113+t42.*t61.*t124),-t160.*t173.*(t164.*(t87+t89+t91+imag(t212).*(1.0./2.0))+t159.*t170.*(t94+t96+t98-real(t212).*(1.0./2.0))),t216.*(-t210+t221+t197.*t213.*2.0).*(1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[3, 13]);
