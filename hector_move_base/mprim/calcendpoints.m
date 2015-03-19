clc;

startangle = [0, pi/8, pi/4]; % 16 directions over the full circle
turnangle = -pi/2:pi/2:pi/2; % 5 different ending directions per starting direction
L = 16;

num_angles = length(turnangle);
endpts_0 = zeros(num_angles,2);



endpts_0(:,1) = L * sin(turnangle(:))./turnangle(:);
endpts_0(:,2) = L * (1-cos(turnangle(:)))./turnangle(:);
endpts_0(2,1) = 1;
endpts_0(2,2) = 0;

plot(endpts_0(:,2), endpts_0(:,1), 'o');
axis([-8, 8, -8, 8]);

for gamma = startangle;
    gamma/(pi/8)
    180*gamma/pi
R = [cos(gamma), -sin(gamma); sin(gamma), cos(gamma)];

endpts_r = zeros(num_angles,2);

for i = 1:num_angles
endpts_r(i,:) = endpts_0(i,:) * R;
end

round_endpts_r = round(endpts_r)

endpts_r;
end