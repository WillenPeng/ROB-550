clear all; close all;
step = 0.1;
D2R = pi/180;
figure
hold on
<<<<<<< HEAD
for q0 = -pi:0.01:pi
    for q1 = (-121*D2R):step:(127*D2R)
        for q2 = (-121*D2R):step:(127*D2R)
            for q3 = (-121*D2R):step:(107*D2R)
                [x,y,z] = FK(q0,q1,q2,q3);
                plot3(x,y,z,'ro')
            end
=======
for q0 = -pi:step:pi
    for q1 = -121*D2R:step:127*D2R
        for q2 = -121*D2R:step:127*D2R
%             for q3 = -121*D2R:step:107*D2R
            [x,y,z] = FK(q0,q1,q2,0);
            plot(sqrt(x^2+y^2),z,'ro')
            [x,y,z] = FK(q0,q1,q2,pi/2);
            plot(sqrt(x^2+y^2),z,'bo')
%             end
>>>>>>> 86e46bbe8450533f8358200539b04ffefcde25c3
        end
    end
end

xlabel('d_w [mm]')
ylabel('Z_w [mm]')
set(gca,'FontSize', 16)
legend('\phi = 0','\phi = \pi/2')

