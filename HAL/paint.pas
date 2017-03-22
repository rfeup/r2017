unit paint;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  IniPropStorage, BGRABitmap, BGRABitmapTypes, math, utils, laserLoc,state;

const
  ImgWidth = 640;
  ImgHeight = 480;
  PixelSize = 0.0075;
  PixelScale = 1 / PixelSize;

type

  TDistMap = array [0..ImgHeight - 1, 0..ImgWidth - 1] of integer;
  TGradMap = array [0..ImgHeight - 1, 0..ImgWidth - 1] of Single;


  { TFPaint }

  TFPaint = class(TForm)
    IniPropStorage: TIniPropStorage;
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormMouseDown(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure FormMouseMove(Sender: TObject; Shift: TShiftState; X, Y: Integer);
    procedure FormMouseUp(Sender: TObject; Button: TMouseButton;
      Shift: TShiftState; X, Y: Integer);
    procedure FormPaint(Sender: TObject);
  private
    procedure BuildField;
    procedure DrawLaser(R: TPose; var LaserPoints: TLaserPoints);
    procedure GradMapToBGRABitmap(aBitMap: TBGRABitmap; Map: TGradMap;
      factor: double);
    { private declarations }
  public
    bmp, bmp_draw: TBGRABitmap;

    DistMap: TDistMap;
    GradXMap, GradYMap, GradThetaMap: TGradMap;

    fieldX, fieldY: double;
    MachineTopX, MachineTopY, MachineX, MachineY: double;
    mouse_down: boolean;
    mouse_u, mouse_v: integer;
    mouse_x, mouse_Y: double;

    RobotPoints: ArrayOfTPointF;
    RobotXsize, RobotYSize, RobotOffsetX: double;

    function XTopixel(x: double): integer; inline;
    function YTopixel(y: double): integer; inline;

    function pixelToX(u: integer): double;
    function pixelToY(v: integer): double;

    procedure BGRABitmapToDistMap(var Map: TDistMap; aBitMap: TBGRABitmap);
    procedure DistMapToBGRABitmap(aBitMap: TBGRABitmap; Map: TDistMap);
    procedure CalcDistMap(var Map: TDistMap);
    procedure CalcGradMap(var aGradXMap, aGradYMap: TGradMap; Map: TDistMap);
    function ScanDistMap(var Map: TDistMap; v: integer): integer;
    function d_err(d: double): double;
    procedure DrawRobot(R: TPose);
    procedure IterLaser(var R: TPose; LaserPoints: TLaserPoints; FirstIdx, LastIdx: integer; scale: double);
  end;

var
  FPaint: TFPaint;
  RobotPose: TPose;
  c_err: double;

implementation


{ TFPaint }

procedure TFPaint.BuildField;
begin
  bmp.Fill(BGRAWhite);

  // draw field walls
  bmp.Rectangle(XTopixel(-fieldX), YTopixel(-fieldY), XTopixel(fieldX), yTopixel(fieldY), BGRABlack,  BGRA(255,255,255),dmSet);

  // draw Machines
  bmp.Rectangle(XTopixel(MachineTopX), YTopixel(MachineTopY), XTopixel(MachineTopX + MachineX), yTopixel(MachineTopY - MachineY), BGRABlack, BGRAWhite,dmSet);
  bmp.Rectangle(XTopixel(-MachineTopX), YTopixel(MachineTopY), XTopixel(-MachineTopX - MachineX), yTopixel(MachineTopY - MachineY), BGRABlack, BGRAWhite,dmSet);

  ET.Start();
  BGRABitmapToDistMap(DistMap, bmp);
  debug('BGRABitmapToDistMap: ' +  ET.ElapsedStr());

  ET.Start();
  CalcDistMap(DistMap);
  debug('CalcDistMap: ' +  ET.ElapsedStr());

  ET.Start();
  CalcGradMap(GradXMap, GradYMap, DistMap);
  debug('CalcGradMap: ' +  ET.ElapsedStr());

  //DistMapToBGRABitmap(bmp, DistMap);
  //GradMapToBGRABitmap(bmp, GradYMap, 2000);
end;


procedure TFPaint.FormCreate(Sender: TObject);
begin
  fieldX := 1.5;
  fieldY := 1.88/2;

  MachineX := 0.24;
  MachineY := 0.36;
  MachineTopX := -0.50 - MachineX;
  MachineTopY := MachineY / 2;

  c_err := 200;

  SetLength(RobotPoints, 7);
  RobotXsize := 0.3;
  RobotYSize := 0.2;
  RobotOffsetX := 0.1;

  RobotPoints[0].x :=   0;
  RobotPoints[0].y :=   0;

  RobotPoints[1].x :=   RobotXsize / 2 - RobotOffsetX;
  RobotPoints[1].y :=   0;

  RobotPoints[2].x :=   RobotXsize / 2 - RobotOffsetX;
  RobotPoints[2].y := - RobotYsize / 2;

  RobotPoints[3].x := - RobotXsize / 2 - RobotOffsetX;
  RobotPoints[3].y := - RobotYsize / 2;

  RobotPoints[4].x := - RobotXsize / 2 - RobotOffsetX;
  RobotPoints[4].y :=   RobotYsize / 2;

  RobotPoints[5].x :=   RobotXsize / 2 - RobotOffsetX;
  RobotPoints[5].y :=   RobotYsize / 2;

  RobotPoints[6].x :=   RobotXsize / 2 - RobotOffsetX;
  RobotPoints[6].y :=   0;

  bmp_draw := TBGRABitmap.Create(ImgWidth, ImgHeight, BGRABlack);

  bmp := TBGRABitmap.Create(ImgWidth, ImgHeight, BGRAWhite);

  BuildField();
end;




procedure TFPaint.FormDestroy(Sender: TObject);
begin
  bmp.Free;
  bmp_draw.Free;
end;

procedure TFPaint.FormMouseDown(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
var i: integer;
begin
  mouse_down := true;
  mouse_u := X;
  mouse_v := y;
  mouse_x := pixelToX(mouse_u);
  mouse_Y := pixelToY(mouse_v);

  //bmp.DrawLine(x, y, XTopixel(mouse_x + 0.1), YTopixel(mouse_Y + 0.1), BGRA(255, 0, 0, 128), true);
  if ssCtrl in Shift then begin
    Pose_loc.x := mouse_x;
    Pose_loc.y := mouse_y;
    Pose_loc.theta := degtoRad(0);
    RobotState.x :=  Pose_loc.x;
    RobotState.y :=  Pose_loc.y;
    RobotState.teta :=  degtoRad(0);
    //DrawRobot(R);
  end else begin
    //bmp.DrawLine(x, y, x - round(GradXMap[y, x] * 10000), y - round(GradYMap[y, x] * 10000), BGRA(255, 0, 0, 128), true);
    for i := 0 to 10 do
      IterLaser(Pose_loc, FLaserLoc.LaserPoints, laserFirstIdx, laserLastIdx, 0.01);
  end;

  FormPaint(Sender);
end;

procedure TFPaint.FormMouseMove(Sender: TObject; Shift: TShiftState; X,
  Y: Integer);
var xw, yw: double;
begin
  if not mouse_down then exit;
  if ssCtrl in Shift then begin
    xw := pixelToX(x);
    yw := pixelToY(y);
    Pose_loc.theta := atan2(yw -Pose_loc.y, xw  - Pose_loc.x);
    RobotState.teta :=  Pose_loc.theta;
  end;

  FormPaint(Sender);
end;

procedure TFPaint.FormMouseUp(Sender: TObject; Button: TMouseButton;
  Shift: TShiftState; X, Y: Integer);
begin
  mouse_down := false;
end;

procedure TFPaint.FormPaint(Sender: TObject);
begin
  bmp_draw.PutImage(0, 0, bmp, dmSet);
  DrawRobot(RobotPose);
  DrawLaser(RobotPose, FLaserLoc.LaserPoints);
  bmp_draw.Draw(Canvas,0,0,True);
end;

function TFPaint.XTopixel(x: double): integer;
begin
  result := round(x * PixelScale) + ImgWidth div 2;
end;

function TFPaint.YTopixel(y: double): integer;
begin
  result := round(-y * PixelScale) + ImgHeight div 2;
end;

function TFPaint.pixelToX(u: integer): double;
begin
  result := (u - ImgWidth div 2) * PixelSize; // PixelSize = 1 / PixelScale
end;

function TFPaint.pixelToY(v: integer): double;
begin
  result := -(v - ImgHeight div 2) * PixelSize; // PixelSize = 1 / PixelScale
end;


procedure TFPaint.BGRABitmapToDistMap(var Map: TDistMap; aBitMap: TBGRABitmap);
var x, y: integer;
    PPixel: PBGRAPixel;
begin
  PPixel := aBitMap.Data;
  for y := 0 to ImgHeight - 1 do begin
    for x := 0 to ImgWidth - 1 do begin
      map[y, x] := 16 * PPixel^.green;
      inc(PPixel);
    end;
  end;
end;

procedure TFPaint.DistMapToBGRABitmap(aBitMap: TBGRABitmap;  Map: TDistMap);
var x, y, v: integer;
    PPixel: PBGRAPixel;
    Pixel: TBGRAPixel;
begin
  PPixel := aBitMap.Data;
  for y := 0 to ImgHeight - 1 do begin
    for x := 0 to ImgWidth - 1 do begin
      v := map[y, x];
      Pixel.red := v;
      Pixel.green := v;
      Pixel.blue := v;
      PPixel^ := Pixel;
      inc(PPixel);
    end;
  end;
end;



procedure TFPaint.GradMapToBGRABitmap(aBitMap: TBGRABitmap;  Map: TGradMap; factor: double);
var x, y, v: integer;
    PPixel: PBGRAPixel;
    Pixel: TBGRAPixel;
begin
  PPixel := aBitMap.Data;
  for y := 0 to ImgHeight - 1 do begin
    for x := 0 to ImgWidth - 1 do begin
      v := 128 + round(map[y, x] * factor);
      Pixel.red := v;
      Pixel.green := v;
      Pixel.blue := v;
      PPixel^ := Pixel;
      inc(PPixel);
    end;
  end;
end;

procedure TFPaint.CalcDistMap(var Map: TDistMap);
var i, misses: integer;
begin
  misses := 0;
  for i := 0 to 1000 do begin
    if ScanDistMap(Map, i) = 0 then begin
      //debug('max_dist: ' + inttostr(i));
      inc(misses);
      if misses > 2 then break;
    end else begin
      misses := 0;
    end;
  end;
end;


function TFPaint.ScanDistMap(var Map: TDistMap; v: integer): integer;
var x, y: integer;
begin
  result := 0;
  for y := 1 to ImgHeight - 2 do begin
    for x := 1 to ImgWidth - 2 do begin
      if map[y, x] <> v then continue;
      inc(result);
      {map[y, x + 1] := min(map[y, x + 1], 1 + v);
      map[y, x - 1] := min(map[y, x - 1], 1 + v);
      map[y + 1, x] := min(map[y + 1, x], 1 + v);
      map[y - 1, x] := min(map[y - 1, x], 1 + v); }

      map[y, x + 1] := min(map[y, x + 1], 2 + v);
      map[y, x - 1] := min(map[y, x - 1], 2 + v);
      map[y + 1, x] := min(map[y + 1, x], 2 + v);
      map[y - 1, x] := min(map[y - 1, x], 2 + v);

      map[y + 1, x + 1] := min(map[y + 1, x + 1], 3 + v);
      map[y + 1, x - 1] := min(map[y + 1, x - 1], 3 + v);
      map[y - 1, x + 1] := min(map[y - 1, x + 1], 3 + v);
      map[y - 1, x - 1] := min(map[y - 1, x - 1], 3 + v);
    end;
  end;
end;

function TFPaint.d_err(d: double): double;
var c2: double;
begin
  c2 :=  c_err * c_err;
  result := 1 - c2 / (c2 + d * d);
end;

procedure TFPaint.DrawRobot(R: TPose);
var u, v, i, n: integer;
    RPoints: ArrayOfTPointF;
    rx, ry: double;
begin
  n := length(RobotPoints);
  SetLength(RPoints, n);
  for i := 0 to n - 1 do begin
    RotateAndTranslate(rx, ry, RobotPoints[i].x, RobotPoints[i].y, R.x, R.y, R.theta);
    RPoints[i].x := XTopixel(Rx);
    RPoints[i].y := YTopixel(Ry);
    //Flaserloc.Memo.Lines.add(format('%g, %g', [RPoints[i].x, RPoints[i].y]));
  end;
  bmp_draw.DrawPolyLineAntialias(RPoints, BGRA(0, 128, 0, 255), 0.5, true);
  bmp_draw.EllipseAntialias(RPoints[0].x, RPoints[0].y, 3, 3, BGRA(0, 128, 0, 255), 0.5);
end;

procedure TFPaint.DrawLaser(R: TPose; var LaserPoints: TLaserPoints);
var u, v, i, n: integer;
    LPoints: ArrayOfTPointF;
    rx, ry: double;
    c, c_ok, c_bad: TBGRAPixel;
begin
  if not assigned(LaserPoints) then exit;
  //n := laserLastIdx - laserFirstIdx + 1;

  c_ok := BGRA(128, 0, 0, 255);
  c_bad := BGRA(0, 128, 0, 255);
  n := length(LaserPoints);
  //SetLength(LPoints, n);
  for i := 0 to n - 1 do begin
  // for i := laserFirstIdx to laserLastIdx do begin
    RotateAndTranslate(rx, ry, LaserPoints[i].x, LaserPoints[i].y, R.x, R.y, R.theta);
    if LaserPoints[i].d < 0.01 then begin
      c := c_bad;
    end else begin
      c := c_ok;
    end;
    //bmp_draw.EllipseAntialias(XTopixel(Rx), YTopixel(Ry), 4, 4, BGRA(128, 0, 0, 255), 0.5);
    bmp_draw.Rectangle(XTopixel(Rx) - 2, YTopixel(Ry) - 2, XTopixel(Rx) + 2,  YTopixel(Ry) + 2, c, dmSet);
    //LPoints[i].x := XTopixel(Rx);
    //LPoints[i].y := YTopixel(Ry);
    //LPoints[i - laserFirstIdx].x := XTopixel(Rx);
    //LPoints[i - laserFirstIdx].y := YTopixel(Ry);
    //Flaserloc.Memo.Lines.add(format('%g, %g', [RPoints[i].x, RPoints[i].y]));
  end;
  //bmp_draw.DrawPolyLineAntialias(LPoints, BGRA(128, 0, 0, 255), 0.5, true);
end;


procedure TFPaint.IterLaser(var R: TPose; LaserPoints: TLaserPoints; FirstIdx, LastIdx: integer; scale: double);
var u, v, i, n: integer;
    rx, ry: double;
    dx, dy, dtheta: double;
    ct, st: double;
    gradX, gradY: double;
begin
  dx := 0;
  dy := 0;
  dtheta := 0;
  st := sin(R.theta);
  ct := cos(R.theta);
  R.err := 0;
  n := 0;

  for i := FirstIdx to LastIdx do begin
    if LaserPoints[i].d < 0.1 then continue;
    //if (i and 1) = 0 then continue;
    //RotateAndTranslate(rx, ry, LaserPoints[i].x, LaserPoints[i].y, R.x, R.y, R.theta);
    RotateAndTranslate(rx, ry, LaserPoints[i].x, LaserPoints[i].y, R.x, R.y, st, ct);
    u := XTopixel(rx);
    v := YTopixel(ry);
    if (u > 0) and (u < ImgWidth -1) and
       (v > 0) and (v < ImgHeight -1) then begin

      //dx := dx - GradXMap[v, u];
      //dy := dy + GradYMap[v, u];
      //dtheta := dtheta - GradXMap[v, u] * ( - LaserPoints[i].x * st - LaserPoints[i].y * ct)
      //                 + GradYMap[v, u] * (   LaserPoints[i].x * ct - LaserPoints[i].y * st);
      gradX := GradXMap[v, u];
      gradY := GradYMap[v, u];

      dx := dx - GradX;
      dy := dy + GradY;
      dtheta := dtheta - GradX * ( - LaserPoints[i].x * st - LaserPoints[i].y * ct)
                       + GradY * (   LaserPoints[i].x * ct - LaserPoints[i].y * st);
      R.err := R.err + DistMap[v, u];
      inc(n);
    end;
  end;
  R.x := R.x + scale * dx;
  R.y := R.y + scale * dy;
  R.theta := R.theta + pi * scale * dtheta;
  if n > 0 then R.err := R.err / n;
end;


procedure TFPaint.CalcGradMap(var aGradXMap, aGradYMap: TGradMap; Map: TDistMap);
var x, y, i: integer;
begin
  for y := 1 to ImgHeight - 2 do begin
    for x := 1 to ImgWidth - 2 do begin
      aGradXMap[y, x] := (d_err(map[y, x + 1]) - d_err(map[y, x - 1])) / 2;
      aGradYMap[y, x] := (d_err(map[y + 1, x]) - d_err(map[y - 1, x])) / 2;
    end;
  end;
end;



initialization
  {$I paint.lrs}

end.
