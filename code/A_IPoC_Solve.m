% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : N/A                                                %
%   Output : State vector solution                              %
%                                                               %
% -------------------------- Content -------------------------- %

for k=1:(length(tt)-1)
    % ------------- Control command ------------- % (x_k == GT | x_e == x^est)
    u_k(:,k)= u_inp( x_e(:,k) );
    % -------------- True process --------------- %
    x_k(:,k+1) = f_RK45(@A_IPoC_fx, x_k(:,k), u_k(:,k), dt) + w_k(); 
    
    % ------------ State estimation ------------- %
    % --------------- Prediction ---------------- %
    x_e(:,k+1) = A_k*x_e(:,k) + B_k*u_k(:,k);     % Linear predictor (x'=[A-BK]x)
    P_k = A_k*P_k*A_k' + Q;                       % Error state covariance    
    
    % ----------------- Update ------------------ %
    if mod(k,rho) == 0
        C_k = h_x(v_idx, n_dim);                  % Projection: position, vel, gyros
        y_meas = C_k*x_k(:,k) + v_k();            % Outputs (Noisy measurements)
        y_pred = C_k*x_e(:,k);                    % Predicted measurements
        y_res  = y_meas - y_pred;                 % Measurements residual

        [x_e(:,k+1), P_k] = KF_update(x_e(:,k), P_k, y_res, C_k, R); % Kalman filter (loop-closure)
        y_k(v_idx,k) = y_meas;
    end
    x_std(:,k+1) = sqrt(diag(P_k));
end