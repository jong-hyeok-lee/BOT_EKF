function rmse_values = update_rmse(Target_pos, Esti_Target_pos, i, rmse_values, dt, h_rmse)
    % RMSE 계산
    rmse = sqrt(mean(sum((Target_pos(1:i, :) - Esti_Target_pos(1:i, :)).^2, 2)));
    rmse_values = [rmse_values, rmse];
    time_step = i * dt; % 초 단위로 계산
    time_steps = linspace(dt, time_step, length(rmse_values)); % 1초 단위로 RMSE 그래프를 업데이트

    % 실시간 RMSE 그래프 업데이트
    set(h_rmse, 'XData', time_steps, 'YData', rmse_values);
end
