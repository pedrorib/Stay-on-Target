% !!! UNTESTED !!!
function [L, R, M] = GetMouseKeyStatus()
if ~ispc
  error('Running under Windows only.');
end

if ~libisloaded('user32')
   loadlibrary('user32.dll', 'user32.h');
end

L = calllib('user32', 'GetAsyncKeyState', int32(1)) ~= 0;
R = calllib('user32', 'GetAsyncKeyState', int32(2)) ~= 0;
M = calllib('user32', 'GetAsyncKeyState', int32(4)) ~= 0;
end
