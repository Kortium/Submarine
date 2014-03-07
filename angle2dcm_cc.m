function DCM = angle2dcm_cc(angles)
DCMZ = eye(3,3);
DCMY = eye(3,3);
DCMX = eye(3,3);

DCMZ(1,1) = cos(angles(1));
DCMZ(1,2) = -sin(angles(1));
DCMZ(2,1) = sin(angles(1));
DCMZ(2,2) = cos(angles(1));

DCMY(1,1) = cos(angles(2));
DCMY(1,3) = sin(angles(2));
DCMY(3,1) = -sin(angles(2));
DCMY(3,3) = cos(angles(2));

DCMX(2,2) = cos(angles(3));
DCMX(2,3) = -sin(angles(3));
DCMX(3,2) = sin(angles(3));
DCMX(3,3) = cos(angles(3));

DCM = DCMZ*DCMY*DCMX;

end