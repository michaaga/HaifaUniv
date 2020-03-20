
cmd = 'forward';
speed = 50;

CarRemoteCmd(cmd, speed);
% Ask server to do sth, use in running mode
%
% 	Post requests to server, server will do what client want to do according to the url.
% 	This function for running mode
%
% 	Args:
% 		# ============== Back wheels =============
% 		'bwready' | 'forward' | 'backward' | 'stop'
%
% 		# ============== Front wheels =============
% 		'fwready' | 'fwleft' | 'fwright' |  'fwstraight'
%
% 		# ================ Camera =================
% 		'camready' | 'camleft' | 'camright' | 'camup' | 'camdown'
