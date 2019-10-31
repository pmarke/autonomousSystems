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
            set(obj.ax,'XLim',[-30,30]);
            set(obj.ax,'YLim',[-30,30]);
            
            % Initialize map
            obj.InitMap();
            
            
        end
        
        function InitMap(obj)
        
            % Make sure that the landmarks are nx2. If not, change it
            size_landmarks = size(obj.landmarks);
            if (size_landmarks(2) > size_landmarks(1))
                obj.landmarks = obj.landmarks';
            end
            
            % Plot the landmarks
            obj.landmark_handle = plot(obj.landmarks(:,1),obj.landmarks(:,2),'*m');
            hold on
            
            % Plot true data
            obj.robot_truth_handle = plot(obj.X_tr(1,:),obj.X_tr(2,:),'g');    
          
        end
        
        function Update(obj,mean,x,r,bearing)
            % point where n is the number of particles
            % mean: The mean of all of the particles. [x_mean,
            % y_mean,th_mean]
            % r: The range measurements
            % th: The bearing measurements
            % flag_draw_all_points: If true, all the pts will be drawn,
            % else only the mean point will be drawn
            
            % Check dimension of incoming parameters and correct
            radius = 0.5;
            
            if isempty(obj.robot_mean_handle)
                h1 = rectangle('Position',[mean(1)-radius mean(2)-radius 2*radius 2*radius],'Curvature',[1,1],'FaceColor','b');
                h2 = line([mean(1),mean(1)+2*radius*cos(mean(3))],[mean(2),mean(2)+2*radius*sin(mean(3))],'Color','k');
                obj.robot_mean_handle = [h1,h2];
                
            else
                    set(obj.robot_mean_handle(1),'Position',[mean(1)-radius mean(2)-radius 2*radius 2*radius]);
                    set(obj.robot_mean_handle(2),'XData',[mean(1),mean(1)+2*radius*cos(mean(3))],'YData',[mean(2),mean(2)+2*radius*sin(mean(3))]);
            end
            
            if isempty(obj.robot_truth_handle_2)
                h1 = rectangle('Position',[x(1)-radius x(2)-radius 2*radius 2*radius],'Curvature',[1,1],'FaceColor','g');
                h2 = line([x(1),x(1)+2*radius*cos(x(3))],[x(2),x(2)+2*radius*sin(x(3))],'Color','k');
                obj.robot_truth_handle_2 = [h1,h2];
                
            else
                    set(obj.robot_truth_handle_2(1),'Position',[x(1)-radius x(2)-radius 2*radius 2*radius]);
                    set(obj.robot_truth_handle_2(2),'XData',[x(1),x(1)+2*radius*cos(x(3))],'YData',[x(2),x(2)+2*radius*sin(x(3))]);
            end
            
            obj.drawMeasurements(x,r,bearing)
            
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
                   
                obj.measurement_true_handle =plot(landmarks_seen(:,1),landmarks_seen(:,2),'pm','LineWidth',3);
                obj.measurement_est_handle = plot(pts_meas(:,1),pts_meas(:,2),'pc','LineWidth',3);
            else
                set(obj.measurement_true_handle,'XData',landmarks_seen(:,1),'YData',landmarks_seen(:,2));
                set(obj.measurement_est_handle,'XData',pts_meas(:,1),'YData',pts_meas(:,2));
            end
            
                 
        end
 
    end
end

