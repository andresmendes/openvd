function output = VelControl4DOF(input,~)
    % Current velocity
    vel = input(5);
    % Reference velocity
    velRef = 8;

    % Control gain
    K = 100000;

    output = K * (velRef - vel);
