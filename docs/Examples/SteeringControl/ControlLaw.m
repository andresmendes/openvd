function controlEffort = ControlLaw(sysData,~)
    % sysData are the informations of the system necessary to generate the control effort
    % controlEffort is the control input of the system

    % In this case, the sysData contains the simple vehicle state variables
    x = [sysData(2);sysData(3);sysData(5);sysData(6)];

    % Vehicle longitudinal position
    X = sysData(1);

    % Lateral displacement
    LateralDisp = 3.6;

    % Reference signal (Double lane change)
    if X <= 15
        r = 0;
    end
    if X > 15 && X <= 70
        r = LateralDisp;
    end
    if X > 70
        r = 0;
    end

    % Control gains
    K = [0.7936    6.6882    1.6107    0.5090];
    u = - K*x + K(1)*r;

    % Saturation of the control effort
    if abs(u) < 42*pi/180
        controlEffort = u;
    else
        controlEffort = sign(u)*42*pi/180;
    end
