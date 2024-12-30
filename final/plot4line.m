%Esti_Target Target UUV1 UUV2 실제 궤적 그리기
function RealTrajec = plot4line(Target,UUV1,UUV2,UUV3)
    figure;
    plot3(Target(:, 1), Target(:, 2),Target(:,3), 'k.', 'LineWidth', 2); hold on;
    plot3(UUV1(:, 1), UUV1(:, 2),UUV1(:, 3), 'g-', 'LineWidth', 2);
    plot3(UUV2(:, 1), UUV2(:, 2),UUV2(:, 3), 'b-', 'LineWidth', 2);
    plot3(UUV3(:, 1), UUV3(:, 2),UUV3(:,3), '-y', 'LineWidth', 2);

    xlabel('X 위치');
    ylabel('Y 위치');
    zlabel('Z 위치')
    title('실제 궤적(m)'); 
    legend('Target', 'UUV1', 'UUV2','UUV3');
end