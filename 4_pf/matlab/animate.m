function animate(t,x,xh,num_meas,m_pos,chi)


% define persistent variables 
persistent landmark_handle
persistent particle_handle
persistent true_handle
persistent est_handle

% first time function is called, initialize plot and persistent vars
if isempty(landmark_handle)
    figure(10), clf
    hold on
    landmark_handle = drawLandmarks(num_meas,m_pos,landmark_handle);
    true_handle = drawTrue(x, true_handle);
    est_handle = drawEstimate(xh, est_handle);
    particle_handle=drawParticles(chi, particle_handle);
    axis([-20, 20, -20,20]);
    axis square


% at every other time step, redraw base and rod
else 
    true_handle = drawTrue(x, true_handle);
    est_handle = drawEstimate(xh, est_handle);
    particle_handle=drawParticles(chi, particle_handle);
end




end

function handle = drawLandmarks(num_meas,m_pos, handle)

if (isempty(handle))
    for ii = 1:num_meas
        px = m_pos(1,ii);
        py = m_pos(2,ii);
        d = 0.5;
        h = rectangle('Position',[px-d/2 py-d/2 d d],'FaceColor',[0 .5 .5]);
        handle = [handle,h];
    end
else
    
    
end

end

function handle = drawParticles(x, handle)


   

if (isempty(handle))
    handle = line(x(1,:),x(2,:),'Color','r','Marker','o','LineStyle','none','MarkerFaceColor','k','MarkerSize',2);
else
 
    set(handle,'XData',x(1,:),'YData',x(2,:));
   
    
end

end


function handle = drawTrue(x, handle)

r = 0.5;

if (isempty(handle))
    
    
    h1 = rectangle('Position',[x(1)-r x(2)-r 2*r 2*r],'Curvature',[1,1],'FaceColor','g');
    h2 = line([x(1),x(1)+2*r*cos(x(3))],[x(2),x(2)+2*r*sin(x(3))],'Color','g');
    handle = [h1,h2];
    
else
    set(handle(1),'Position',[x(1)-r x(2)-r 2*r 2*r]);
    set(handle(2),'XData',[x(1),x(1)+2*r*cos(x(3))],'YData',[x(2),x(2)+2*r*sin(x(3))]);
end

end

function handle = drawEstimate(x, handle)

r = 0.5;

if (isempty(handle))
    
    
    h1 = rectangle('Position',[x(1)-r x(2)-r 2*r 2*r],'Curvature',[1,1],'FaceColor','r');
    h2 = line([x(1),x(1)+2*r*cos(x(3))],[x(2),x(2)+2*r*sin(x(3))],'Color','k');
    handle = [h1,h2];
    
else
    set(handle(1),'Position',[x(1)-r x(2)-r 2*r 2*r]);
    set(handle(2),'XData',[x(1),x(1)+2*r*cos(x(3))],'YData',[x(2),x(2)+2*r*sin(x(3))]);
end


end