function [gps_week, gps_sow, gps_dow] = local2gps(local_t, utc_gap)

% SYNTAX:
%   [gps_week, gps_sow, gps_dow] = local2gps(local_t, utc_gap);
%
% INPUT:
%   local_t = [year, month, day, hour, min, sec]
%   utc_gap = local_time - utc_time (in hour), example: [Zurich UTC+2/+1]
%  
% OUTPUT:
%   gps_week = GPS week
%   gps_sow  = GPS seconds of week
%   gps_dow  = GPS day of week
%
% DESCRIPTION:
%   Conversion from local time to GPS time.

%% Leap seconds DATES
stepdates = [...
    'Jan 6 1980'
    'Jul 1 1981'
    'Jul 1 1982'
    'Jul 1 1983'
    'Jul 1 1985'
    'Jan 1 1988'
    'Jan 1 1990'
    'Jan 1 1991'
    'Jul 1 1992'
    'Jul 1 1993'
    'Jul 1 1994'
    'Jan 1 1996'
    'Jul 1 1997'
    'Jan 1 1999'
    'Jan 1 2006'
    'Jan 1 2009'
    'Jul 1 2012'
    'Jul 1 2015'
    'Jan 1 2017'];
% add more dates below

%% Convert Steps to datenums and make step offsets
stepdates = datenum(stepdates)'; %step date coversion
steptime = (0:length(stepdates)-1)'./86400; %corresponding step time (sec)

%% Arg Checking
utc_t = local_t;
utc_t(:,4)=local_t(:,4)-utc_gap; % in hour
utc_t = datenum(utc_t);  % in date

if ~isempty(find(utc_t < stepdates(1)))%utc_t must all be after GPS start date
    error('Input dates must be after 00:00:00 on Jan 6th 1980') 
end

%% Array Sizing
sz = size(utc_t);
utc_t = utc_t(:);

utc_t = repmat(utc_t,[1 size(stepdates,2)]);
stepdates = repmat(stepdates,[size(utc_t,1) 1]);

%% Conversion
gps_date = utc_t(:,1)   + steptime(sum((utc_t - stepdates) >= 0,2));

%% Reshape Output Array
gps_date = reshape(gps_date,sz);

%% Get GPS week, seconds/days of the week
gps_start_datenum = 723186; %This is datenum([1980,1,6,0,0,0])

%number of days since the beginning of GPS time 
delta_date   = gps_date - gps_start_datenum; % in day

gps_week = floor(delta_date/7);                        %GPS week
gps_dow  = floor(delta_date - gps_week*7);     %GPS day of week
gps_sow  = (delta_date - gps_week*7)*86400; %GPS seconds of week

