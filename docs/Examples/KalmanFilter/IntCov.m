function dP = IntCov(~,P,F,G,Q)
    % Covariance
    P = reshape(P, [6 6])';
    dPMat = F*P + P*F' + G*Q*G';
    dP = reshape(dPMat',[1 36])';
end
