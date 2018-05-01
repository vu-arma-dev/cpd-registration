v = VideoWriter('test.avi','Uncompressed AVI');
open(v);
k=1;
while k<180
    APP_RegAprToRobot_PSM_Hamlyn('iter max',k);
    set(gcf, 'Position', [100, 100, 1280, 1080]);
    iter_string = sprintf('Iteration: %0.0f',k);
    text('Interpreter','latex',...
        'String',iter_string,...
        'Position',[10 30 -165],...
        'FontSize',20);
    frame = getframe(gcf);
    writeVideo(v,frame);
    if k<30
        k = k+1;
    else
        k = k+2;
    end
end
close(v)