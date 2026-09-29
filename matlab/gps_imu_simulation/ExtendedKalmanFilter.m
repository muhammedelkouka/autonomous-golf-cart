classdef ExtendedKalmanFilter < handle
%EXTENDEDKALMANFILTER  5-state planar EKF for GNSS/IMU fusion.
%
%   State  x = [px; py; v; theta; bg; ba]
%       px, py : planar position                     [m]
%       v      : forward speed                       [m/s]
%       theta  : heading (wrapped to [-pi,pi))       [rad]
%       bg     : gyroscope bias                      [rad/s]
%       ba     : accelerometer bias                  [m/s^2]
%
%   The accelerometer bias MUST be a state when there is no odometry aiding.
%   A constant bias integrating into velocity is not representable as white
%   process noise, so a filter that omits it cannot track the resulting
%   velocity ramp and diverges outright (thousands of metres over a 480 s
%   run). With the bias estimated, position updates observe it through its
%   quadratic signature and the filter stays consistent.
%
%   Gyroscope rate is an INPUT to predict().  Speed is a MEASUREMENT
%   supplied through updateSpeed().  This is the change that makes the
%   Jacobian consistent with the propagation: because the model now takes
%   v from the state, dpx/dv = cos(theta)*dt is correct.  The previous
%   implementation propagated position using v from the input while
%   assigning x(3) = v and claiming F(1,3) = cos(theta)*dt, which put
%   position-speed cross-covariance into P that the model did not have.
%
%   Qc is a CONTINUOUS-TIME process noise density.  predict() scales it by
%   dt internally.  Do NOT pre-scale it.  Passing a Q that was pre-scaled
%   for one rate and then calling predict() at a different rate was the
%   original tuning error.
%
%   Diagnostics: every position update records the innovation, its
%   covariance S, and the normalised innovation squared (NIS).  For a
%   correctly tuned 2-D position update, NIS should average ~2 and lie
%   inside the 95% chi-square interval [0.051, 7.378] about 95% of the
%   time.  If it does not, the filter is mistuned and its RMSE means
%   nothing.  Check this before reporting any number from this filter.

    properties
        x       % (5x1) state
        P       % (5x5) covariance
        Qc      % (5x5) continuous-time process noise density
    end

    properties (SetAccess = private)
        innovHistory = zeros(2,0);   % logged innovations
        nisHistory   = zeros(1,0);   % logged NIS values
        nAccepted    = 0;
        nRejected    = 0;
    end

    methods
        function obj = ExtendedKalmanFilter(x0, P0, Qc)
            validateattributes(x0, {'numeric'}, {'vector','numel',6,'finite'});
            validateattributes(P0, {'numeric'}, {'size',[6 6],'finite'});
            validateattributes(Qc, {'numeric'}, {'size',[6 6],'finite'});
            obj.x  = x0(:);
            obj.P  = (P0 + P0.')/2;
            obj.Qc = Qc;
        end

        function predict(obj, omega_meas, accel_meas, dt)
        %PREDICT  Propagate state and covariance forward by dt seconds.
        %   omega_meas : measured yaw rate                [rad/s]
        %   accel_meas : measured longitudinal accel      [m/s^2]
        %   dt         : ACTUAL elapsed time of the caller's loop [s]
            if dt <= 0
                return
            end

            v     = obj.x(3);
            theta = obj.x(4);
            bg    = obj.x(5);
            ba    = obj.x(6);
            omega = omega_meas - bg;
            accel = accel_meas - ba;

            obj.x(1) = obj.x(1) + v*cos(theta)*dt;
            obj.x(2) = obj.x(2) + v*sin(theta)*dt;
            obj.x(3) = v + accel*dt;
            obj.x(4) = ExtendedKalmanFilter.wrapPi(theta + omega*dt);
            % bg, ba are random walks: unchanged in the mean

            F = eye(6);
            F(1,3) =  cos(theta)*dt;   F(1,4) = -v*sin(theta)*dt;
            F(2,3) =  sin(theta)*dt;   F(2,4) =  v*cos(theta)*dt;
            F(3,6) = -dt;
            F(4,5) = -dt;

            obj.P = F*obj.P*F.' + obj.Qc*dt;
            obj.P = (obj.P + obj.P.')/2;
        end

        function accepted = updatePosition(obj, z, R, gateNIS)
        %UPDATEPOSITION  Correct with a 2-D position fix.
        %   z        : [zx; zy] measured position [m]
        %   R        : (2x2) measurement covariance
        %   gateNIS  : optional chi-square gate; Inf or [] disables gating.
        %              7.378 is the 97.5th percentile for 2 dof.
            if nargin < 4 || isempty(gateNIS), gateNIS = Inf; end

            H  = [1 0 0 0 0 0;
                  0 1 0 0 0 0];
            z  = z(:);
            nu = z - H*obj.x;                    % innovation
            S  = H*obj.P*H.' + R;                % innovation covariance
            S  = (S + S.')/2;
            nis = nu.' * (S \ nu);

            obj.innovHistory(:,end+1) = nu;
            obj.nisHistory(end+1)     = nis;

            if nis > gateNIS
                obj.nRejected = obj.nRejected + 1;
                accepted = false;
                return
            end

            K = (obj.P*H.') / S;
            obj.x = obj.x + K*nu;
            obj.x(4) = ExtendedKalmanFilter.wrapPi(obj.x(4));

            % Joseph form: stays positive-definite over long runs, unlike
            % the (I-KH)P form used previously.
            I = eye(6);
            A = I - K*H;
            obj.P = A*obj.P*A.' + K*R*K.';
            obj.P = (obj.P + obj.P.')/2;

            obj.nAccepted = obj.nAccepted + 1;
            accepted = true;
        end

        function updateSpeed(obj, v_meas, R_v)
        %UPDATESPEED  Correct the speed state with a scalar speed measurement.
            H  = [0 0 1 0 0 0];
            nu = v_meas - H*obj.x;
            S  = H*obj.P*H.' + R_v;
            K  = (obj.P*H.') / S;

            obj.x = obj.x + K*nu;
            obj.x(4) = ExtendedKalmanFilter.wrapPi(obj.x(4));

            I = eye(6);
            A = I - K*H;
            obj.P = A*obj.P*A.' + K*R_v*K.';
            obj.P = (obj.P + obj.P.')/2;
        end

        function s = diagnostics(obj)
        %DIAGNOSTICS  Summary used to decide whether the filter is tuned.
            nis = obj.nisHistory;
            lo  = 0.0506;   % chi2inv(0.025, 2)
            hi  = 7.3778;   % chi2inv(0.975, 2)
            s.meanNIS      = mean(nis, 'omitnan');
            s.fracInGate   = mean(nis > lo & nis < hi, 'omitnan');
            s.nAccepted    = obj.nAccepted;
            s.nRejected    = obj.nRejected;
            s.innovMean    = mean(obj.innovHistory, 2, 'omitnan');
            s.innovStd     = std(obj.innovHistory, 0, 2, 'omitnan');
            s.isConsistent = abs(s.meanNIS - 2) < 0.7 && s.fracInGate > 0.85;
        end
    end

    methods (Static, Access = private)
        function a = wrapPi(a)
            a = mod(a + pi, 2*pi) - pi;
        end
    end
end
