function transform = Config2LRSupportFeetTransform (robot, config, numberOfFrameInHalf)
    % Config to left and right support feet homogeneous tansformation
    %
    % Input
    %   - robot | rigidbodytree         : A N-DOF biped robot
    %   - config | M-by-N matrix        : Configuration path of the robot
    %   - numberOfFrameInHalf | int     : The number of "animation frame" in a half walking cycle
    % Output
    %   - transform | cell array        : Left and right support feet homogeneous tansformation

    isLeftFoot = true; % Left foot is support feet at begining
    index = 1;
    supportFeetChanged = 1 + numberOfFrameInHalf/2; % The moment of support feet changed
    supportFeetTransform = trvec2tform([6.75 .75 0])*eul2tform([0 0 0]); % Initial transformation of support feet
    initFrame2SupportFrame = eul2tform([pi/2 0 0])*eul2tform([0 -pi/2 -pi/2]);
    % initFrame2SupportFrame = eul2tform([0 -pi/2 -pi/2])
    transform{index} = supportFeetTransform; % Set first transformation
    for i = 2 : size(config, 1)
        if i == supportFeetChanged + 1 % Calculate support feet transformation only   
            if isLeftFoot % Left foot is the supporting foot
                tf = getTransform(robot, config(i, :), 'body12', 'body11'); % Find right feet transformation
            else
                tf = getTransform(robot, config(i, :), 'body11', 'body12');
            end

            % Update
            supportFeetChanged = supportFeetChanged + numberOfFrameInHalf; % Swap left and right feet
            supportFeetTransform = supportFeetTransform*initFrame2SupportFrame*tf/initFrame2SupportFrame;
            index = index + 1;
            isLeftFoot = ~isLeftFoot;

            transform{index} = supportFeetTransform*initFrame2SupportFrame;

            % show(robot, config(i, :));
            % F{index-1} = getframe(gcf);
        end
    end
end