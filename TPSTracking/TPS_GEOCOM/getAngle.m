function [Hz, V] = getAngle(TPSport)
% Request - send a command to get a simple angle measurement
    fprintf('\nTMC_GetAngle5 - returning a simple angle measurement\n');
    req='%R1Q,2107:';
    fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
    out = fscanf (TPSport);

% Extract the returned values
    data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*)','names');

% Check output and store result
    if str2num(data.RC) == 0
        Hz = str2double (data.Hz);
        V = str2double (data.V); 

% Print the values
        fprintf('Hz = %.4f [gon]; V = %.4f [gon]\n', Hz * 200/pi, V * 200/pi);
    else
        fprintf('GRC_Return-Code= %.0f; Check the GRC return-code in manual!\n', data.RC)
    end

end