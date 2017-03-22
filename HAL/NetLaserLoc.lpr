program NetLaserLoc;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Interfaces, // this includes the LCL widgetset
  Forms, laserloc, lnetvisual, TAChartLazarusPkg, bgrabitmappack,
  etpackage, paint, structs, CameraUDP, hal;

//{$IFDEF WINDOWS}{$R NetLaserLoc.rc}{$ENDIF}

{$R *.res}

begin
  //{$I NetLaserLoc.lrs}
  Application.Initialize;
  Application.CreateForm(TFLaserLoc, FLaserLoc);
  Application.CreateForm(TFPaint, FPaint);
  Application.CreateForm(TFHal, FHal);
  Application.Run;
end.

