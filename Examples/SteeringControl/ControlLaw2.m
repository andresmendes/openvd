function output = ControlLaw2(input,t)
    % The input is the simple vehicle state variables
    x = [input(2);input(3);input(5);input(6)];
    % Reference

    if t<= 3
        r = 2;
    else
        r = 0;
    end

    % Control gain
    K = [1.0000    8.3851    4.1971    0.6460];
    u = - K*x + K(1)*r;

    % Saturation at 70 deg
    if abs(u) < 70*pi/180
        output = u;
    else
        output = sign(u)*70*pi/180;
    end
