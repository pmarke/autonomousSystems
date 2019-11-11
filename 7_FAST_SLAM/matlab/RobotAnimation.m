classdef RobotAnimation < handle
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Location of all of the landmarks
        landmarks  
        
        % True robot pose data
        X_tr
        
        % Handles to figure, axes, and other figure objects
        fig
        ax
        landmark_handle;
        landmark_est_handle;
        ellipse_handle;
        robot_mean_handle;
        robot_truth_handle;
        robot_truth_handle_2;
        measurement_est_handle;
        measurement_true_handle;
        
    end
    
    methods
        function obj = RobotAnimation(landmarks,X_tr)
            % Initialize properties
            obj.landmarks = landmarks;
            obj.X_tr = X_tr;
            obj.fig = figure(10);
            clf;
            obj.ax = obj.fig.CurrentAxes;
            axis manual
            xlim([-15 15])
            ylim([-15 15])
            
            % Initialize map
            obj.InitMap();
            
            
        end
        
        function InitMap(obj)
        
            % Make sure that the landmarks are nx2. If not, change it
            size_landmarks = size(obj.landmarks);
            if (size_landmarks(1) == 2)
                obj.landmarks = obj.landmarks';
            end
            
            % Plot the landmarks
            obj.landmark_handle = plot(obj.landmarks(:,1),obj.landmarks(:,2),'*m');
            hold on
            
            % Plot true data
            obj.robot_truth_handle = plot(obj.X_tr(1,:),obj.X_tr(2,:),'g');    
          
        end
        
        function Update(obj,mean,x,std_p,r,th)
            % point where n is the number of particles
            % mean: The estimated state
            % x:    The true state
            % r: The range measurements
            % th: The bearing measurements
            % flag_draw_all_points: If true, all the pts will be drawn,
            % else only the mean point will be drawn
            
            % Check dimension of incoming parameters and correct
            radius = 0.5;
            
            est_landmarks = mean(4:end);
            est_landmarks = reshape(est_landmarks,2,[]);
            
            if isempty(obj.robot_mean_handle)
                h1 = rectangle('Position',[mean(1)-radius mean(2)-radius 2*radius 2*radius],'Curvature',[1,1],'FaceColor','b','HandleVisibility','off');
                h2 = line([mean(1),mean(1)+2*radius*cos(mean(3))],[mean(2),mean(2)+2*radius*sin(mean(3))],'Color','k','HandleVisibility','off');
                obj.landmark_est_handle = line(est_landmarks(1,:),est_landmarks(2,:),'Color','b','Marker','*','LineStyle','none','HandleVisibility','off');
                obj.robot_mean_handle = [h1,h2];
                
            else
                    set(obj.robot_mean_handle(1),'Position',[mean(1)-radius mean(2)-radius 2*radius 2*radius]);
                    set(obj.robot_mean_handle(2),'XData',[mean(1),mean(1)+2*radius*cos(mean(3))],'YData',[mean(2),mean(2)+2*radius*sin(mean(3))]);
                    set(obj.landmark_est_handle,'XData',est_landmarks(1,:),'YData',est_landmarks(2,:));
            end
            
            % Draw landmark covariances
            landmark_size = size(obj.landmarks);
            if isempty(obj.ellipse_handle)
                for ii = 1:landmark_size(1)
                    [X,Y] = obj.getEllipsePoints(std_p(2*ii-1),std_p(2*ii),mean(2+2*ii),mean(3+2*ii));
                    h = line(X,Y,'Color','b','HandleVisibility','off');
                    obj.ellipse_handle = [obj.ellipse_handle,h];
                end
                
            else
                for ii = 1:landmark_size(1)
                    [X,Y] = obj.getEllipsePoints(std_p(2*ii-1),std_p(2*ii),mean(2+2*ii),mean(3+2*ii));
                    set(obj.ellipse_handle(ii),'XData',X,'YData',Y);
                end
                    
            end 
            
            if isempty(obj.robot_truth_handle_2)
                h1 = rectangle('Position',[x(1)-radius x(2)-radius 2*radius 2*radius],'Curvature',[1,1],'FaceColor','g','HandleVisibility','off');
                h2 = line([x(1),x(1)+2*radius*cos(x(3))],[x(2),x(2)+2*radius*sin(x(3))],'Color','k','HandleVisibility','off');
                obj.robot_truth_handle_2 = [h1,h2];
                
            else
                    set(obj.robot_truth_handle_2(1),'Position',[x(1)-radius x(2)-radius 2*radius 2*radius]);
                    set(obj.robot_truth_handle_2(2),'XData',[x(1),x(1)+2*radius*cos(x(3))],'YData',[x(2),x(2)+2*radius*sin(x(3))]);
            end
            
%             obj.drawMeasurements(mean,r,th)
            
        end
        
        function drawMeasurements(obj, mean,r,bearing)
            
            % Get all landmarks that were seen
            landmarks_seen = obj.landmarks(~isnan(r),:);
        
            
            % Remove the nan
            rs = r(~isnan(r));
            bs = bearing(~isnan(bearing));
            
            
            % Convert measurements to points
            pts_meas = zeros(length(rs),2);
            for ii = 1:length(rs)
                pts_meas(ii,1) = mean(1)+rs(ii)*cos(mean(3)+bs(ii));
                pts_meas(ii,2) = mean(2)+rs(ii)*sin(mean(3)+bs(ii));
            end
            
            if(isempty(obj.measurement_true_handle))
                   
                obj.measurement_true_handle =plot(landmarks_seen(:,1),landmarks_seen(:,2),'pm','LineWidth',3,'HandleVisibility','off');
                obj.measurement_est_handle = plot(pts_meas(:,1),pts_meas(:,2),'pc','LineWidth',3,'HandleVisibility','off');
            else
                set(obj.measurement_true_handle,'XData',landmarks_seen(:,1),'YData',landmarks_seen(:,2));
                set(obj.measurement_est_handle,'XData',pts_meas(:,1),'YData',pts_meas(:,2));
            end
            
                 
        end
        
        function drawEstimateTrack(obj,x_est_history)
            
           obj.robot_truth_handle = plot(x_est_history(1,:),x_est_history(2,:),'b-');
           xlim([-25 25])
           ylim([-25 25])
           legend('landmarks','true pose', 'est pose');
        end
        
        function [X,Y] = getEllipsePoints(obj,std_x,std_y,mx,my)
            t = linspace(0,2*pi,10);
            X = 2*std_x*cos(t)+mx;
            Y = 2*std_y*sin(t)+my;
        end
 
    end
end

