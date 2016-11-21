function output = VelControl(input,~)
    % Current velocity
    vel = input(5);
    % Reference velocity
    velRef = 8;

    % Control gain
    K = 400000;

    output = K * (velRef - vel);
