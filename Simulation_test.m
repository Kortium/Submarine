function Z = Simulation_test(NUMBER_LOOPS,wp,lm)
INIT_FILE;
QE= Q; RE= R; if SWITCH_INFLATE_NOISE, QE= 2*Q; RE= 8*R; end % inflate estimated noises (ie, add stabilising noise)
full_way = 0;
ftag = 1:length(lm);
i=1;
idf_v=[];
idf=[];
plot3(wp(1,:),wp(2,:),wp(3,:), '-*g','MarkerSize',10)
xlim([-150 150]), ylim([-150 150]), zlim([0 100]);
xlabel('metres'), ylabel('metres'), zlabel('metres')
hold on
plot3(lm(1,:),lm(2,:),lm(3,:), 'r.','MarkerSize',10)
xlabel('metres'), ylabel('metres'), zlabel('metres')
hold on
h.xt = plot3(0,0,0,'b');
h.pd = plot3(0,0,0,'r');
h.pdnc = plot3(0,0,0,'m');
h.vis = plot3(0,0,0,'*b','MarkerSize',10);
% h.vis_m_pos = plot3(0,0,0,'*m','MarkerSize',10);
[xsb,ysb,zsb] = sphere(20);
sphere_mass=[];
t = 0:0.2:1;
[Xc,Yc,Zc]=cylinder(t);
q = angle2quat(0, pi/2, 0);
XYZ_veh = quatrotate(q,[(Xc(:).*3),(Yc(:)).*3,((Zc(:)).*9)-4]);
q = angle2quat(0, 3*pi/2, 0);
XYZ = quatrotate(q,[(Xc(:).* 28.87),(Yc(:)).*57.73,(Zc(:)).*100]);
Xc_veh = reshape(XYZ_veh(:,1),6,21);
Yc_veh = reshape(XYZ_veh(:,2),6,21);
Zc_veh = reshape(XYZ_veh(:,3),6,21);
Xc = reshape(XYZ(:,1),6,21);
Yc = reshape(XYZ(:,2),6,21);
Zc = reshape(XYZ(:,3),6,21);
h.vision_cyl = surf(Xc.*0,Yc.*0,Zc.*0);
h.veh_cyl = surf(Xc_veh.*0,Yc_veh.*0,Zc_veh.*0);
set(h.vision_cyl,'FaceColor','b','EdgeColor','b','EdgeAlpha',0.2);
set(h.veh_cyl,'FaceColor','r','EdgeColor','r','EdgeAlpha',0.4);
alpha(h.vision_cyl,0.4);
alpha(h.vision_cyl,0.2);
xt=xtrue(1:3)';
pd=[0;0;100];
pdnc=[0;0;100];
da_table= zeros(1,size(lm,3));
set(fig, 'name', 'EKF-SLAM Simulator')
axis equal
grid on
hold on
% pause(10);
while iwp ~= 0
    i=i+1;    
    % compute true data
    [wPsi,wTeta,wRoll,iwp]= compute_rotation(xtrue, wp, iwp, AT_WAYPOINT, wPsi, RATEP, MAXP, wTeta, RATET, MAXT,wRoll, RATER, MAXR, dt);
    W=[wRoll,wTeta,wPsi];  
    if iwp==0 && NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    xtrue = vehicle_model(xtrue, V,W, dt); 
     [Vn,Wn] = add_control_noise(V,W,Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step    
     [x,P]= predict (x,P, Vn,Wn,Q,dt);
%     [xnc,Pnc]= predict (xnc,Pnc, Vn,Wn,Q,dt);
    
    

    
     % EKF update step
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum=0;
        [z,idf_v]= observations_submerged(xtrue, lm, ftag, rmax);  
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
        
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(x,z,idf_v, da_table);
        else
            [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT);
        end
        
         if SWITCH_USE_IEKF == 1
            [x,P]= update_iekf(x,P,zf,RE,idf, 5);
        else
            [x,P]= update(x,P,zf,RE,idf, SWITCH_BATCH_UPDATE,xt(3,end)); 
        end
        [x,P]= augment(x,P, zn,RE); 
    end
%     disp(P);
    xt=[xt(1,:) xtrue(1); xt(2,:) xtrue(2); xt(3,:) xtrue(3)];
    pd=[pd(1,:) x(1); pd(2,:) x(2); pd(3,:) x(3)];
    pdnc = [pdnc(1,:) xnc(1); pdnc(2,:) xnc(2); pdnc(3,:) xnc(3)];
    full_way = full_way + V*dt;
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:), 'zdata', xt(3,:));   
    set(h.pd, 'xdata', pd(1,:), 'ydata', pd(2,:), 'zdata', pd(3,:)); 
    set(h.pdnc, 'xdata', pdnc(1,:), 'ydata', pdnc(2,:), 'zdata', pdnc(3,:)); 
%     if size(idf_v)>0
%         set(h.vis, 'xdata', lm(1,idf_v(:)), 'ydata', lm(2,idf_v(:)), 'zdata', lm(3,idf_v(:)));    
%     end        
    set(h.vis, 'xdata', x(14:3:end), 'ydata', x(15:3:end), 'zdata', x(16:3:end));
    sphere_mass = add_sphere (x,P,sphere_mass,xsb,ysb,zsb);
    sphere_refresh(x,P,sphere_mass,xsb,ysb,zsb);   
    XYZ = quatrotate([-xtrue(4),xtrue(5),xtrue(6),xtrue(7)],[Xc(:),Yc(:),(Zc(:))]);
    Xcyl = reshape(XYZ(:,1),6,21);
    Ycyl = reshape(XYZ(:,2),6,21);
    Zcyl = reshape(XYZ(:,3),6,21);
    set(h.vision_cyl,'xdata', Xcyl+xt(1,end), 'ydata', Ycyl+xt(2,end), 'zdata', Zcyl+xt(3,end));  
    XYZ = quatrotate([-xtrue(4),xtrue(5),xtrue(6),xtrue(7)],[Xc_veh(:),Yc_veh(:),(Zc_veh(:))]);
    Xcyl = reshape(XYZ(:,1),6,21);
    Ycyl = reshape(XYZ(:,2),6,21);
    Zcyl = reshape(XYZ(:,3),6,21);
    set(h.veh_cyl,'xdata', Xcyl+xt(1,end), 'ydata', Ycyl+xt(2,end), 'zdata', Zcyl+xt(3,end));
    drawnow
    Z = x;
end

function sphere_mass = add_sphere (x,P,sphere_mass,base_sphere_x,base_sphere_y,base_sphere_z)
    k = ((length(x)-13)/3) - length(sphere_mass);
    if k>0
        for j = ((length(x)-13)/3-k+1):((length(x)-13)/3)
            X = x(14+3*(j-1));
            Y = x(15+3*(j-1));
            Z = x(16+3*(j-1));
            px = P(14+3*(j-1),14+3*(j-1));
            py = P(15+3*(j-1),15+3*(j-1));
            pz = P(16+3*(j-1),16+3*(j-1));
            sphere_mass(length(sphere_mass)+1) = mesh(base_sphere_x.*px+X,base_sphere_y.*py+Y,base_sphere_z.*pz+Z);
            alpha(sphere_mass(end),0.2);
            drawnow            
        end
    end
        
function sphere_refresh(x,P,sphere_mass,base_sphere_x,base_sphere_y,base_sphere_z)
    for j=1:length(sphere_mass)
        X = x(14+3*(j-1));
        Y = x(15+3*(j-1));
        Z = x(16+3*(j-1));
        px = P(14+3*(j-1),14+3*(j-1));
        py = P(15+3*(j-1),15+3*(j-1));
        pz = P(16+3*(j-1),16+3*(j-1));
        set(sphere_mass(j),'xdata',base_sphere_x*px+X,'ydata',base_sphere_y*py+Y,'zdata',base_sphere_z*pz+Z);
        drawnow
    end        
