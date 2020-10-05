clear; clc;


% CONSTANTS
prism_type = 0; % Leica Circular Prism
target_type = 0; % Use reflector
atr_state = 1; % ATR state on
hz_tol = 1.57079e-05; % Horizontal tolerance (high)
v_tol = 1.57079e-05; % Vertical tolerance (high)
distmode = 2 % Default distance mode (from BAP_SetMeasPrg)
%prism1_h = 
%prism1_v = 


% Open port, connect to TPS
obj = instrfind; if ~isempty(obj); delete(obj); end
TPSport = serial('/dev/ttyUSB0', 'BaudRate', 115200, 'Parity','none', 'DataBits', 8, 'StopBits', 1, 'Terminator', 'CR/LF', 'TimeOut', 60);
fopen (TPSport);
fprintf('TPS is connected\n\n');

% Change prism type
prism_out = SetPrismType(prism_type, TPSport);
if str2num(prism_out) == 0
    fprintf("Prism type changed\n");
else
    fprintf("Error in changing prism type\n");
end


% Change target type (EDM mode)
req=['%R1Q,17021:',num2str(target_type)];
fprintf (TPSport, req);
out = fscanf (TPSport);
data = regexp(out,':(?<RC>[^,]*)','names');
if str2num(data.RC) == 0
    fprintf("EDM mode changed\n");
else
    fprintf("Error in changing EDM mode type\n");
end

% Change ATR state
req=['%R1Q,18005:',num2str(atr_state)];
fprintf (TPSport, req);
out = fscanf (TPSport);
data = regexp(out,':(?<RC>[^,]*)','names');
if str2num(data.RC) == 0
    fprintf("ATR state changed\n");
else
    fprintf("Error in changing ATR state type\n");
end

% Set tolerances
req=['%R1Q,9007:',num2str(hz_tol), ',', num2str(v_tol)];
fprintf (TPSport, req);
out = fscanf (TPSport);
data = regexp(out,':(?<RC>[^,]*)','names');
if str2num(data.RC) == 0
    fprintf("Horizontal and vertical tolerances set\n");
else
    fprintf("Error in changing tolerances\n");
end

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

% prism1 Hz = 251.3598 [gon]; V = 99.9020 [gon]
% prism2 Hz = 198.1351 [gon]; V = 93.0201 [gon]
% prism3 Hz = 50.3610 [gon]; V = 93.2096 [gon]
% prism4 Hz = 374.7677 [gon]; V = 100.0592 [gon]
% prism5 Hz = 274.7734 [gon]; V = 99.9686 [gon]

% Set tolerances
req=['%R1Q,17017:', num2str(distmode)];
fprintf (TPSport, req);
out = fscanf (TPSport);
data = regexp(out,':(?<RC>[^,]*),(?<Hz>[^,]*),(?<V>[^,]*),(?<D>[^,]*)','names');
fprintf('Horz: %s Vert: %s Dist: %s', data.Hz, data.V, data.D);


%% AUT_MakePositioning - turning the telescope to a specified position
% %R1Q,9027:Hz,V,PosMode,ATRMode,0
% %R1P,0,0:RC
Hz1=0;
V1=1.5706;

% Request - send a command to the totalstation to set Hz=0, V=100gon
fprintf('\nAUT_MakePositioning - turning the telescope to a specified position (Hz=%sgon, V=%sgon)\n',num2str(Hz1),num2str(V1) );

req=['%R1Q,' sprintf('9027:%s,%s,0,0',num2str(Hz1),num2str(V1))];
fprintf (TPSport, req); 

% Reply - read response data into the sting-variable out
out = fscanf (TPSport); 

% Extract the returned values
data = regexp(out,':(?<RC>[^,]*)','names');

if str2num(data.RC) == 0
    fprintf('Telescope turned to Hz=%.4f and V=%.4f\n\n',Hz1,V1 );
end

fprintf("Done.\n");

