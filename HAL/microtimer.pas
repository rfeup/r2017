unit microtimer;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils; 

function getMiliTime: int64;
function getMicroTime: int64;

var
  PerformanceFrequency: int64;

implementation

{$IFDEF MSWINDOWS}
uses windows;
{$ELSE}
uses unix;
{$ENDIF}


function getMicroTime: int64;
{$IFDEF MSWINDOWS}
var tv: int64;
begin
  queryPerformanceCounter(tv);
  result := tv div PerformanceFrequency;
end;
{$ELSE}
var tv: ttimeval;
begin
  fpgettimeofday(@tv, nil);
  result := int64(tv.tv_sec) * 1000000 + tv.tv_usec;
end;
{$ENDIF}

function getMiliTime: int64;
begin
  result := getMicroTime() div 1000;
end;


initialization

{$IFDEF MSWINDOWS}
  QueryPerformanceFrequency(PerformanceFrequency);
  PerformanceFrequency := PerformanceFrequency div 1000000;
{$ENDIF}


end.
