step = 1;
D2R = pi/180;
figure
hold on
for q0 = -pi:step:pi
    for q1 = -121*D2R:step:127*D2R
        for q2 = -121*D2R:step:127*D2R
            for q3 = -121*D2R:step:107*D2R
                [x,y,z] = FK(q0,q1,q2,q3);
                plot3(x,y,z,'ro')
            end
        end
    end
end

