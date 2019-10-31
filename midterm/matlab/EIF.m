classdef EIF < handle
        
    properties
        mu = zeros(3,1);    % Estimated state
        W  = zeros(2,2);    % Measurement noise covariance
        ep = zeros(2,2);    % Process noise covariance
        zeta = zeros(3,1);  % Transformed estimated state
        P = zeros(3,3);     % Error covariance
        Omega = zeros(3,3); % Inverse error covariance
        
        mu_history = [];
        P_history = [];
        
    end
    
    methods
        function obj = EIF(mu0,P)
            obj.mu = mu0;
            obj.P = P;
            obj.Omega = inv(obj.P);
            obj.zeta = obj.Omega*obj.mu;
            
            obj.W = [0.2^2, 0;...
                      0,    0.1^2];
                  
            obj.ep = [0.15^2, 0;...
                        0,     0.1^2];
                    
            obj.UpdateHistory();
            
        end
        
        function Predict(obj,Ts,u)
            % Predict step for EIF.
            % Ts is the time step
            % u: is the input vector u=[v,w]'
            
            % Compute jacobians
            Fk = obj.F(Ts,u);
            Gk = obj.G(Ts);
            
            % Predict inverse error covariance and estimated transformed state
            obj.Omega = inv(Fk/obj.Omega*Fk'+Gk*obj.ep*Gk');
%             Omega = obj.Omega
            obj.zeta = obj.Omega*obj.f(Ts,u);          
            
            % Update mu
            obj.mu = obj.Omega\obj.zeta;
            
        end
        
        function Update(obj,r,phi,ell)
            % The update phase of the EIF.
            % r: is the measured range to the target
            % phi: is the relative angle to the target
            % ell: the location of the target. ell = [mx,my]
            z = [r;phi];
            
            % Compute jacobian
            Hk = obj.H(ell);
            
            % Update inverse error covariance and estimated transformed state
            obj.Omega = obj.Omega + Hk'*inv(obj.W)*Hk;
            obj.zeta  = obj.zeta + Hk'*inv(obj.W)*(z-obj.h(ell)+Hk*obj.mu);
            
            % Update error covariance and estimated state
            obj.P = inv(obj.Omega);
            obj.mu = obj.Omega\obj.zeta;
        end
        
%         function Predict(obj,Ts,u)
%             % Predict step for EIF.
%             % Ts is the time step
%             % u: is the input vector u=[v,w]'
%             
%             % Compute jacobians
%             Fk = obj.F(Ts,u);
%             Gk = obj.G(Ts);
%             
%             % Predict inverse error covariance and estimated transformed state
%             obj.P = Fk*obj.P*Fk' + Gk*obj.ep*Gk'; 
%         
%             
%             % Update mu
%             obj.mu = obj.f(Ts,u);
%             
%         end
        
%         function Update(obj,r,phi,ell)
%             % The update phase of the EIF.
%             % r: is the measured range to the target
%             % phi: is the relative angle to the target
%             % ell: the location of the target. ell = [mx,my]
%             z = [r;phi];
%             
%             % Compute jacobian
%             Hk = obj.H(ell);
%                         
%             % Update inverse error covariance and estimated transformed state
%             S = Hk*obj.P*Hk'+obj.W;
%             K = obj.P*Hk'*inv(S);
%             er = z-obj.h(ell);
%             
%             obj.mu = obj.mu+K*(er);
%             obj.P = (eye(3)-K*Hk)*obj.P;
%             
%         end
        function X=f(obj,Ts,u)
            % The system function
            % Ts: time step
            % u:  input to the system
            x = obj.mu(1);     % x position of UAV
            y = obj.mu(2);     % y position of UAV
            th = obj.mu(3);    % heading of UAV
            v = u(1);          % velocity
            w = u(2);          % angular rate
            
            X=[x + v*cos(th)*Ts;...
               y + v*sin(th)*Ts;...
               th + w*Ts];
        end
        
        function z=h(obj,ell)
            % Observation funciton. It computes the range and relative
            % bearing to the landmark ell
            % ell: position of landmark
            mx = ell(1);   % x position of landmark
            my = ell(2);   % y position of landmark
            x = obj.mu(1);     % x position of UAV
            y = obj.mu(2);     % y position of UAV
            th = obj.mu(3);    % heading of UAV
            
            if (th > pi)
                th = th-2*pi;
            elseif(th < -pi)
                th = th+2*pi;
            end
                
            
            q = (mx-x)^2 + (my-y)^2;
            z = [sqrt(q);...
                atan2(my-y,mx-x)-th];
            if (z(2) > pi)
                z(2) = z(2)-2*pi;
            elseif(z(2) < -pi)
                z(2) = z(2)+2*pi;
            end
            
        end
        
        function F_out = F(obj,Ts,u)
            % Returns the jacobian of the system funciton
            % The system function
            % Ts: time step
            % u:  input to the system
            th = obj.mu(3);    % heading of UAV
            v = u(1);          % velocity
            
            F_out = [1, 0, -v*sin(th)*Ts;...
                     0, 1,  v*cos(th)*Ts;...
                     0, 0,  1];            
        end
        
        function G_out = G(obj,Ts)
            % Returns the jacobian of the process noise function
            % Ts: time step
            th = obj.mu(3);    % heading of UAV
            G_out = [cos(th)*Ts, 0;...
                     sin(th)*Ts, 0;...
                         0,      Ts];
        end
        
        function H_out = H(obj,ell)
            % Return the jacobian of the observation function                        
            % ell: position of landmark
            mx = ell(1);   % x position of landmark
            my = ell(2);   % y position of landmark
            x = obj.mu(1);     % x position of UAV
            y = obj.mu(2);     % y position of UAV
            
            q = (mx-x)^2 + (my-y)^2;
            
            H_out = [-(mx-x)/sqrt(q), -(my-y)/sqrt(q), 0;...
                        (my-y)/q,        -(mx-x)/q,    -1];
        end
        
        function UpdateHistory(obj)
            obj.mu_history = [obj.mu_history,obj.mu];
            obj.P_history =  [obj.P_history,obj.P];
        end
    end
end
