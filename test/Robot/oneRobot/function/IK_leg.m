function theta = IK_leg(robot, CoM, r_lh)
%Inverse Kinemic of a leg (CoM to foot)

ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];
n = length(CoM);
theta = zeros(6, n);

% CoM to inertial
tform = cell(1, n);
frame0 = eye(4);%trvec2tform([0 -rb.L(1) rb.L(2)]); % joint1 to CoM
frame3 = eul2tform([-pi/2 -pi/2 0]); % Lfoot to Lfoot_endeffector
for i = 1 : n
    frame1 = trvec2tform(CoM(1:3, i)')*eul2tform(flip(CoM(4:6, i)')); % inertial to CoM
    frame2 = trvec2tform(r_lh(1:3, i)')*eul2tform(flip(r_lh(4:6, i)')); % inertial to Lfoot
    tform{i} = frame0/frame1*frame2*frame3;
end

configSol = [0 0 0 0 0 0];
disp(['solving IK of a leg, i = 1~' num2str(n)])
for i = 1 : n
    if mod(i, 100) == 0
        disp(['i = ', num2str(i)])
    end
    END_EFFECTOR = robot.BodyNames{robot.NumBodies};
    [configSol, ~] = ik(END_EFFECTOR, tform{i}, weights, configSol);
    % tform = getTransform(robot,configSol,'body7','base')
    theta(:, i) = configSol;
end
end

