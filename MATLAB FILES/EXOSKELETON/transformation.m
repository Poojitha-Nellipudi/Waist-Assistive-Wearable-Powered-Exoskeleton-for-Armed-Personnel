function matrix =  transformation(dh)
   %[alpha, a, d, theta] = dh;
   alpha = dh(1);
   a = dh(2);
   d = dh(3);
   theta = dh(4);
   matrix = [cos(theta) -sin(theta) 0 a; 
        sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
        sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) d*cos(alpha);
        0 0 0 1];
end