function angle = AngleRangeCorrector(angle)
    if angle > 180
        while angle > 180
            angle = angle - 2*180;
        end
    elseif angle < -180
        while angle < -180
            angle = angle + 2*180;
        end
    end
end
