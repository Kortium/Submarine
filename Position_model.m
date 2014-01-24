function true = Position_model(z,xt)

true =[];
for i = 1:size(z,2)
 teta= z(1,i); ksi = z(2,i); rho= z(3,i);
 true =[true, [xt(1,end) + rho*cos(ksi)*cos(teta); xt(2,end) + rho*cos(ksi)*sin(teta); xt(3,end) + rho*sin(ksi)]]; 
end
if size(z,2)==0
    true=[0;0;0];
end
end
