%% TPSControlTemplate.m
%% Total Station Control via GeoCOM

clear; clc;

%% Open TS port
obj = instrfind; if ~isempty(obj); delete(obj); end
TPSport = serial('COM3', 'BaudRate', 57600, 'Parity','none', 'DataBits', 8, 'StopBits', 1, 'Terminator', 'CR/LF', 'TimeOut', 60);
fopen (TPSport);
fprintf('TPS is connected\n\n');

%% Basic commands (few examples)
% Chapter of Leica TPS1200 GeoCOM Reference Manual
% ASCII-Request
% ASCII-Response


%% 15.6.11 TMC_GetStation - getting the station coordinates of the instrument
% %R1Q,2009:
% %R1P,0,0:RC,E0[double],N0[double],H0[double],Hi[double]
% Request - send a command to the total station to get the station coordinates
fprintf('\nTMC_GetStation - getting the station coordinates of the instrument\nTo execute press Enter\n');
pause
req='%R1Q,2009,1:';

fprintf(TPSport, req);

% Reply - read response data into the sting-variable out
out = fscanf(TPSport);

% Extract the returned values
data = regexp(out, ':(?<RC>[^,]*),(?<E0>[^,]*),(?<N0>[^,]*),(?<H0>[^,]*)', 'names');
    
% Check output and store result
if str2num(data.RC) == 0
   Station(1) = str2double(data.E0);
   
   Station(2) = str2double(data.N0);
   Station(3) = str2double(data.H0);
    
   fprintf('The coordinates of the TPS are: E0 = %.3f N0 = %.3f H0 = %.3f\n', Station(1),Station(2),Station(3));
else
   fprintf('GRC_Return-Code= %.0f; Check the GRC return-code in manual!\n', data.RC)
end

 
%% 15.4.4 TMC_GetAngle5 - returning a simple angle measurement
% %R1Q,2107:Mode[long]
% %R1P,0,0:RC,Hz[double],V[double]

% Request - send a command to get a simple angle measurement
fprintf('\nTMC_GetAngle5 - returning a simple angle measurement\nTo execute press Enter\n');
pause
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


%% AUT_MakePositioning - turning the telescope to a specified position
% %R1Q,9027:Hz,V,PosMode,ATRMode,0
% %R1P,0,0:RC
Hz1=300.0000;
V1=100.0000;

% Request - send a command to the totalstation to set Hz=0, V=100gon
fprintf('\nAUT_MakePositioning - turning the telescope to a specified position (Hz=%sgon, V=%sgon)\nTo execute press Enter\n',num2str(Hz1),num2str(V1) );
pause
req=['%R1Q,' sprintf('9027:%s,%s,0,0',num2str(Hz1*pi/200),num2str(V1*pi/200))];
fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
out = fscanf (TPSport); 

% Extract the returned values
data = regexp(out,':(?<RC>[^,]*)','names');

if str2num(data.RC) == 0
    fprintf('Telescope turned to Hz=%.4f and V=%.4f\n\n',Hz1,V1 );
end

%% 15.4.4 TMC_GetAngle5 - returning a simple angle measurement
% %R1Q,2107:Mode[long]
% %R1P,0,0:RC,Hz[double],V[double]

% Request - send a command to get a simple angle measurement
fprintf('\nTMC_GetAngle5 - returning a simple angle measurement\nTo execute press Enter\n');
pause
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


% Close TPS port
fclose(TPSport);
fprintf('\nTPS is disconnected\n\n');