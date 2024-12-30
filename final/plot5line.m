%% Target UUV1 UUV2 실제 궤적 그리기
function UUV123Target = plot5line(Target,UUV1,UUV2,UUV3,Esti_Tar)
    figure;
    plot3(Target(:, 1), Target(:, 2),Target(:,3), 'k.', 'LineWidth', 2); hold on;
    plot3(UUV1(:, 1), UUV1(:, 2),UUV1(:, 3), 'g-', 'LineWidth', 2);
    plot3(UUV2(:, 1), UUV2(:, 2),UUV2(:, 3), 'b-', 'LineWidth', 2);
    plot3(UUV3(:, 1), UUV3(:, 2),UUV3(:, 3), '-y', 'LineWidth', 2);
    plot3(Esti_Tar(:, 1), Esti_Tar(:, 2),Esti_Tar(:, 3), 'r-', 'LineWidth', 1);

    xlabel('X 위치(m)');
    ylabel('Y 위치(m)');
    zlabel('Z 위치(m)');
    title('실제 궤적(m)'); 
    legend('Target', 'UUV1', 'UUV2','UUV3','추정궤적','Location','eastoutside');
end