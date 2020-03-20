
function res = CarRemoteCmd(cmd, t, run)

global HOST;
HOST = '10.150.1.143';
global PORT;
PORT = '8000';

global BASE_URL;
BASE_URL = ['http://' HOST ':' PORT '/' ];

% Ask server to do sth, use in running mode
%
% 	Post requests to server, server will do what client want to do according to the url.
% 	This function for running mode
%
% 	Cmd Format: [x y car_angle front_wheel_velocity front_wheel_angle acceleration]

%car wheels cycle is ~22cm, for 1m/s -> 4.55 rounds per sec.
%Alg Max speed bounded at 5m/s.

% set the url include action information
request = matlab.net.http.RequestMessage;


if run == 0
    
    UrlCmd = [ '&x='         mat2str(cmd(:,1)) ...
               '&y='         mat2str(cmd(:,2)) ...
               '&car_angle=' mat2str(round(cmd(:,3)*180/pi + 90)) ...
               '&fw_vel='    mat2str(round((cmd(:,4) / 5) * 100)) ...
               '&fw_angle='  mat2str(round(cmd(:,5)*180/pi + 90)) ...
               '&acc='       mat2str(cmd(:,6)) ...
               '&t='         mat2str(t)];

    tmpUrl = [BASE_URL 'run/?action=MATLAB_SET_CMD' UrlCmd ];
    
else
    %use this command to run all the previous queued commands.
    tmpUrl = [BASE_URL 'run/?action=MATLAB_RUN_CMD'];  
    
end 

url = matlab.net.URI(tmpUrl);

% post request with url
res = sendRequest(url, request);

if(~res.StatusCode)
    disp 'failed to send HTTP Req!';
end

end

