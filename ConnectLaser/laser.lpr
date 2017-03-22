program laser;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  heaptrc,
  Interfaces, // this includes the LCL widgetset
  Forms,
lasers, Utils, LResources, TAChartLazarusPkg, lnetvisual, genlist, SysUtils
  { you can add units after this };

{$IFDEF WINDOWS}{$R laser.rc}{$ENDIF}

var heap_report_file: string;

begin
  {$I laser.lrs}
  Application.Initialize;
  Application.CreateForm(TFLasers, FLasers);
  Application.Run;

  heap_report_file := 'heap_report.txt';
  if FileExists(heap_report_file) then DeleteFile(heap_report_file);
  SetHeapTraceOutput(heap_report_file);
end.

