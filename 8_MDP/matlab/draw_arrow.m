function [] = draw_arrow(x0,y0,scale,angle)

% draw and arrow in a Matlab plot
% angle = 0 corresponds to an arrow pointing up (+y direction)
% increasing angle results in arrow rotating CCW from +y direction
% scale = 1 corresponds to an arrow of length 1
% shift is an (x,y) shift in the location of the tail of the arrow.

    arrow_x = [0 0 -0.2 0 0.2];
    arrow_y = [0 1 0.8 1 0.8];

   
    
    for ii = 1:length(x0)
    
        shift = [x0(ii); y0(ii)];
        th = angle(ii);
        R = [cos(th) sin(th); -sin(th) cos(th)];

        a = R*scale*[arrow_x; arrow_y] + shift;
        a_x = a(1,:);
        a_y = a(2,:);
%         ii

        plot(a_x,a_y,'b');
    
    end

end