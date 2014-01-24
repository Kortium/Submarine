function Z = Simulation_test(NUMBER_LOOPS,wp,lm)
INIT_FILE;
QE= Q; RE= R; if SWITCH_INFLATE_NOISE, QE= 2*Q; RE= 8*R; end % inflate estimated noises (ie, add stabilising noise)
full_way = 0;
ftag = 1:length(lm);
i=1;
plot3(wp(1,:),wp(2,:),wp(3,:), '-*g','MarkerSize',10)
xlabel('metres'), ylabel('metres'), zlabel('metres')
hold on
plot3(lm(1,:),lm(2,:),lm(3,:), 'r.','MarkerSize',10)
xlabel('metres'), ylabel('metres'), zlabel('metres')
xlim([-150 150]), ylim([-150 150]), zlim([0 100]);
hold on
h.xt = plot3(0,0,0,'b');
h.pd = plot3(0,0,0,'r');
h.vis = plot3(0,0,0,'*b','MarkerSize',10);
% h.vis_m_pos = plot3(0,0,0,'*m','MarkerSize',10);
[xsb,ysb,zsb] = sphere(20);
h.sphere = mesh(xsb.*0,ysb.*0,zsb.*0);
alpha(h.sphere,0.2);
t = 0:0.2:1;
[Xc,Yc,Zc]=cylinder(t);
q = angle2quat(0, -pi/2-0.6109, 0);
XYZ = quatrotate(q,[(Xc(:).* 28.87),(Yc(:)).*57.73,(Zc(:)).*100]);
Xc = reshape(XYZ(:,1),6,21);
Yc = reshape(XYZ(:,2),6,21);
Zc = reshape(XYZ(:,3),6,21);
h.vision_cyl = surf(Xc.*0,Yc.*0,Zc.*0);
set(h.vision_cyl,'FaceColor','b','EdgeColor','b','EdgeAlpha',0.2);
alpha(h.vision_cyl,0.2);
xt=[0;0;100];
pd=[0;0;100];
da_table= zeros(1,size(lm,3));
set(fig, 'name', 'EKF-SLAM Simulator')
axis equal
grid on
hold on
% pause(10);
while iwp ~= 0
    i=i+1;    
    % compute true data
    [T,Ps,Rs,iwp]= compute_rotation(xtrue, wp, iwp, AT_WAYPOINT, T, RATET, MAXT, Ps, RATEP, MAXP,Rs, RATER, MAXR, dt);
    W=[T,Ps,Rs];  
    if iwp==0 && NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    xtrue= vehicle_model(xtrue, V,W, dt); 
    [Vn,Wn] = add_control_noise(V,W,Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step    
    [x,P]= predict (x,P, Vn,Wn,Q,dt);
    

    
     % EKF update step
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum=0;
        z=[];
        [z,idf_v]= observations_submerged(xtrue, lm, ftag, rmax);  
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        
        if size(z,2)>1
            qq=0;
        end
        
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(x,z,idf_v, da_table);
        else
            [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT);
        end
        
         if SWITCH_USE_IEKF == 1
            [x,P]= update_iekf(x,P,zf,RE,idf, 5);
        else
            [x,P]= update(x,P,zf,RE,idf, SWITCH_BATCH_UPDATE); 
        end
        [x,P]= augment(x,P, zn,RE); 
    end
    xt=[xt(1,:) xtrue(1); xt(2,:) xtrue(2); xt(3,:) xtrue(3)];
    pd=[pd(1,:) x(1); pd(2,:) x(2); pd(3,:) x(3)];
    full_way = full_way + V*dt;
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:), 'zdata', xt(3,:));   
    set(h.pd, 'xdata', pd(1,:), 'ydata', pd(2,:), 'zdata', pd(3,:)); 
   set(h.vis, 'xdata', x(14:3:end), 'ydata', x(15:3:end), 'zdata', x(16:3:end));
%     true_pos = Position_model(z,xt);
%     set(h.vis_m_pos, 'xdata', true_pos(1,:), 'ydata', true_pos(2,:), 'zdata', true_pos(3,:));
%     set(h.vis, 'xdata', lm(1,idf_v), 'ydata', lm(2,idf_v), 'zdata', lm(3,idf_v));
    [angle1,angle2,angle3] = quat2angle(xtrue(4:7));
    q = angle2quat(-angle1,(-angle2)*cos(-angle1)-pi/6,angle3);
    XYZ = quatrotate(quatinv(xtrue(4:7)),[Xc(:),Yc(:),(Zc(:))]);
    Xcyl = reshape(XYZ(:,1),6,21);
    Ycyl = reshape(XYZ(:,2),6,21);
    Zcyl = reshape(XYZ(:,3),6,21);
    set(h.vision_cyl,'xdata', Xcyl+xt(1,end), 'ydata', Ycyl+xt(2,end), 'zdata', Zcyl+xt(3,end));
    %     [angle1,angle2,angle3] = quat2angle([P(4,4),P(5,5),P(6,6),P(7,7)]);
    %      set(h.sphere, 'xdata', (xsb.*(P(1,1)))+xt(1,end), 'ydata', (ysb.*(P(2,2))+xt(2,end)),'zdata', (zsb.*(P(3,3))+xt(3,end)));    
   drawnow
end
