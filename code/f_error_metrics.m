function f_error_metrics(err, n_dim, tt, str)
% ------------------------ Description ------------------------ %
%                                                               %
%   Input  : True (x_k) and estimated (x_e) state vectors       %
%   Output : Steady-state errors, ITA, and ITAE                 %
%                                                               %
% -------------------------- Content -------------------------- %

[err_pos, err_ang] = deal(err(1,:), err(n_dim/2+1,:));
[e_ss_pos, e_ss_ang] = deal( abs(err_pos(end)), abs(err_ang(end)));

[IAE_pos, IAE_ang] = deal(trapz(tt, abs(err_pos)), trapz(tt, abs(err_ang)));
[ITAE_pos, ITAE_ang] = deal(trapz(tt, tt.*abs(err_pos)), trapz(tt, tt.*abs(err_ang)));

% Display results
fprintf('\n* ----------- The %s results: ----------- *\n', str);
fprintf('IAE_pos: %.2f\t\t IAE_ang: %.2f \n', IAE_pos, IAE_ang);
fprintf('ITAE_pos: %.2f\t\t ITAE_ang: %.2f \n', ITAE_pos, ITAE_ang);
fprintf('e_ss_pos: %.2f\t\t e_ss_ang: %.3f \n', e_ss_pos, e_ss_ang);