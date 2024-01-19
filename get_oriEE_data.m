vert_vec = [0 1];
%you should load 'bodyLine_force.mat' file from mechanics model for yaw and bend joint.m file. (Bending joint data)
for k = 1:81

    ori_yaw = [bodyline_force(k,169,1), bodyline_force(k,169,2)];
    ori_yaw2 = [bodyline_force(k,168,1), bodyline_force(k,168,2)];
    
    vect_yaw = (ori_yaw-ori_yaw2);
    
    oriEE_b(k) = acos(dot(vert_vec,vect_yaw)/(norm(vect_yaw)*norm(vert_vec)));

end
