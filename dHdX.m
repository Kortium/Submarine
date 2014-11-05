function dHdX = dHdX(q1,q2,q3,q4,x,xf,y,yf,z,zf)
%DHDX
%    DHDX = DHDX(Q1,Q2,Q3,Q4,X,XF,Y,YF,Z,ZF)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    06-Nov-2014 01:49:43

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
t65 = q4.*t5;
t66 = q2.*t9;
t67 = q4.*t6;
t68 = q3.*t9;
t69 = q3.*t5;
t70 = q2.*t6;
t71 = q1.*t9;
t72 = q3.*t6;
t73 = q4.*t9;
t74 = imag(t69);
t75 = imag(t70);
t76 = imag(t71);
t77 = t76.*2.0;
t78 = real(t73);
t79 = t78.*2.0;
t80 = q2.*t5;
t81 = imag(t73);
t82 = t81.*2.0;
t83 = real(t69);
t84 = real(t70);
t85 = real(t71);
t86 = imag(t65);
t87 = imag(t66);
t88 = real(t67);
t89 = real(t68);
t90 = q1.*t5;
t91 = imag(t67);
t92 = imag(t68);
t93 = real(t65);
t94 = q1.*t6;
t95 = real(t66);
t96 = t95.*2.0;
t97 = t18-t26+t29;
t98 = t14+t20-t24;
t100 = t12+t16-1.0;
t101 = t9.*t100;
t102 = t21-t22;
t103 = t5.*t102;
t104 = t27+t33;
t105 = t6.*t104;
t106 = t97.^2;
t107 = t98.^2;
t108 = t106+t107;
t109 = sqrt(t108);
t127 = real(t101);
t128 = real(t103);
t129 = real(t105);
t130 = imag(t109);
t99 = t127+t128-t129-t130;
t115 = imag(t101);
t116 = imag(t103);
t117 = imag(t105);
t118 = real(t109);
t110 = t115+t116-t117+t118;
t111 = real(t63);
t112 = t111.*2.0;
t113 = real(t64);
t114 = t113.*2.0;
t119 = 1.0./sqrt(t108);
t120 = t13.*t98.*2.0;
t210 = t25.*t97.*2.0;
t121 = t120-t210;
t122 = t119.*t121;
t123 = imag(t63);
t124 = t123.*2.0;
t125 = imag(t64);
t126 = t125.*2.0;
t131 = t110.^2;
t132 = t99.^2;
t133 = t131+t132;
t134 = 1.0./t133;
t135 = real(t61);
t136 = real(t62);
t137 = t136.*2.0;
t138 = 1.0./t110;
t139 = t17.*t97.*2.0;
t140 = t19.*t98.*2.0;
t141 = t139+t140;
t142 = t119.*t141;
t143 = imag(t61);
t144 = t143.*2.0;
t145 = imag(t62);
t146 = 1.0./t110.^2;
t147 = real(t15);
t148 = t147.*2.0;
t149 = real(t3);
t150 = t149.*2.0;
t151 = imag(t15);
t152 = t151.*2.0;
t153 = imag(t3);
t154 = t153.*2.0;
t155 = t23.*t98.*2.0;
t215 = t28.*t97.*2.0;
t156 = t155-t215;
t157 = t83.*2.0;
t158 = q4.*t5.*2.0;
t193 = q2.*t9.*2.0;
t159 = t158-t193;
t160 = t97.*t159.*2.0;
t161 = q4.*t6.*2.0;
t192 = q3.*t9.*2.0;
t162 = t161-t192;
t217 = t98.*t162.*2.0;
t163 = t160-t217;
t164 = t119.*t163;
t165 = t75.*2.0;
t166 = real(t94);
t167 = t166.*2.0;
t168 = t86.*2.0;
t169 = imag(t94);
t170 = t169.*2.0;
t171 = q2.*t6.*4.0;
t172 = q1.*t9.*2.0;
t216 = q3.*t5.*2.0;
t173 = t171+t172-t216;
t174 = t97.*t173.*2.0;
t175 = q3.*t6.*2.0;
t176 = q4.*t9.*2.0;
t177 = t175+t176;
t218 = t98.*t177.*2.0;
t178 = t174-t218;
t179 = t119.*t178;
t180 = real(t90);
t181 = imag(t90);
t182 = t181.*2.0;
t183 = t91.*2.0;
t184 = q2.*t6.*2.0;
t219 = q3.*t5.*4.0;
t185 = t172+t184-t219;
t186 = t98.*t185.*2.0;
t187 = q2.*t5.*2.0;
t188 = t176+t187;
t189 = t97.*t188.*2.0;
t190 = t186+t189;
t191 = t119.*t190;
t194 = real(t80);
t195 = t194.*2.0;
t196 = real(t72);
t197 = t196.*2.0;
t198 = q1.*t5.*2.0;
t220 = q4.*t6.*4.0;
t199 = t192+t198-t220;
t200 = t97.*t199.*2.0;
t201 = q4.*t5.*4.0;
t202 = q1.*t6.*2.0;
t203 = -t193+t201+t202;
t221 = t98.*t203.*2.0;
t204 = t200-t221;
t205 = t119.*t204;
t206 = imag(t80);
t207 = t206.*2.0;
t208 = imag(t72);
t209 = t208.*2.0;
t211 = t101+t103-t105;
t212 = t211.^2;
t213 = t106+t107+t212;
t214 = 1.0./sqrt(t213);
dHdX = reshape([-t45.*t60.*(t52.*(t54-t55.*2.0-t56.*2.0+t154)+t44.*t57.*(t47+t49+t51+t150-1.0)),t131.*t134.*(t138.*(-t112+t114+imag(t122).*(1.0./2.0))+t99.*t146.*(t124-t126+real(t122).*(1.0./2.0))),t214.*(t120-t210+t102.*t211.*2.0).*(1.0./2.0),-t45.*t60.*(t52.*(t47+t49-t51+t148-1.0)-t44.*t57.*(t54-t55.*2.0+t56.*2.0+t152)),t131.*t134.*(t138.*(t135.*2.0+t137+imag(t142).*(1.0./2.0))-t99.*t146.*(t144+t145.*2.0-real(t142).*(1.0./2.0))),t214.*(t139+t140-t104.*t211.*2.0).*(1.0./2.0),t45.*t60.*(t52.*(t124+t126-t135.*2.0+t137)+t44.*t57.*(t112+t114+t144-t145.*2.0)),-t131.*t134.*(t138.*(t148+t150+imag(t119.*t156).*(1.0./2.0)-1.0)-t99.*t146.*(t152+t154-real(t119.*t156).*(1.0./2.0))),t214.*(-t155+t215+t100.*t211.*2.0).*(1.0./2.0),t45.*t60.*(t52.*(t92.*2.0+t93.*2.0-t96-t183)+t44.*t57.*(t87.*2.0-t88.*2.0+t89.*2.0-t168)),-t131.*t134.*(t138.*(t84.*-2.0+t157+imag(t164).*(1.0./2.0))+t99.*t146.*(t74.*-2.0+t165+real(t164).*(1.0./2.0))),t214.*(t160-t217+t211.*(t184-t216).*2.0).*(-1.0./2.0),t45.*t60.*(t52.*(t82-t84.*4.0-t85.*2.0+t157+t209)+t44.*t57.*(t74.*-2.0+t75.*4.0+t77+t79+t197)),t131.*t134.*(t138.*(t93.*2.0-t95.*4.0+t167+imag(t179).*(1.0./2.0))+t99.*t146.*(t87.*4.0-t168-t170+real(t179).*(1.0./2.0))),t214.*(-t174+t218+t211.*(t158+t202-q2.*t9.*4.0).*2.0).*(-1.0./2.0),t45.*t60.*(t52.*(t74.*-4.0+t77+t79+t165+t195)-t44.*t57.*(t82+t83.*4.0-t84.*2.0-t85.*2.0+t207)),-t131.*t134.*(t138.*(t88.*-2.0+t89.*4.0+t180.*2.0+imag(t191).*(1.0./2.0))-t99.*t146.*(t92.*4.0+t182-t183-real(t191).*(1.0./2.0))),t214.*(t186+t189-t211.*(-t161+t198+q3.*t9.*4.0).*2.0).*(-1.0./2.0),-t45.*t60.*(t52.*(t86.*4.0-t87.*2.0+t88.*4.0-t89.*2.0+t170-t180.*2.0)+t44.*t57.*(t91.*-4.0+t92.*2.0+t93.*4.0-t96+t167+t182)),t131.*t134.*(t138.*(t195+t197-imag(t205).*(1.0./2.0))-t99.*t146.*(t207+t209+real(t205).*(1.0./2.0))),t214.*(t200-t221+t211.*(t175+t187).*2.0).*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[3, 13]);
