function TPSport = connect_tps(COMport, dB)
% connect_tps: connect total station to computer via GeoCOM

    obj = instrfind; 
    if ~isempty(obj)
        delete(obj); 
        fprintf('Cleared serial port to connect TPS\n');
    end
    TPSport = serial(COMport, 'BaudRate', dB, 'Parity','none', 'DataBits', 8, 'StopBits', 1, 'Terminator', 'CR/LF', 'TimeOut', 60);
    fopen (TPSport);
    fprintf('TPS is successfully connected\n');
end