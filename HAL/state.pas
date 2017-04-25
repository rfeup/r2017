unit state;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Graphics, structs, hal;

{$DEFINE FAST_PATH}

const
  maxActions = 4096;

  // safe parameters
(*  defaultSpeed = 0.3;
  warehouseApproachSpeed = 0.2;
  pathRadius = 0.3;
  MachineApproachX = 0.05;
*)
  // unsafe parameters
(*  defaultSpeed = 0.4;
  warehouseApproachSpeed = 0.3;
  pathRadius = 0.4;
  MachineApproachX = 0.1;
*)

  sourceX = -1.11; //-1.13;
  targetX = 1.09; //1.13

  clawCenterDistance = 0.3;

  warehouseHallwayY = 0.53;

  warehouseLeaveSpeed = 0.25;
  defaultFinalSpeed = 0;

  warehouseY = 0.78;
  warehouseX = 0.30;

  warehouseSpace = 0.18;

  DefSpeedSafe = 0.30;

  NormalAdvance = warehouseY - warehouseHallwayY;
  NormalAdvanceX = 0.15;

var
  ActionScript: array[0 .. maxActions-1] of Taction;
  ActionScriptCount: integer;

  PartsScript: array[0 .. NUM_Parts] of TPart;
  PartsScript2: array[0 .. NUM_Parts] of TPart;
  PartsValidation: TPart;

  rebuild:integer;

  RobotState: TRobotState;
  count, count2: integer;
      count3: integer;
  CurStep, ve: integer;
  CurStepStamp: int64;

  defaultSpeed: double;
  warehouseApproachSpeed: double;
  pathRadius: double;
  test: boolean;

  BoxPlaces: array[0..18] of TBoxPlace = (
      (x: sourceX + warehouseSpace * 0; y: warehouseHallwayY; teta: 90; advance: NormalAdvance; kind: bpkSource),
      (x: sourceX + warehouseSpace * 1; y: warehouseHallwayY; teta: 90; advance: NormalAdvance; kind: bpkSource),
      (x: sourceX + warehouseSpace * 2; y: warehouseHallwayY; teta: 90; advance: NormalAdvance; kind: bpkSource),
      (x: sourceX + warehouseSpace * 3; y: warehouseHallwayY; teta: 90; advance: NormalAdvance; kind: bpkSource),
      (x: sourceX + warehouseSpace * 4; y: warehouseHallwayY; teta: 90; advance: NormalAdvance; kind: bpkSource),

      (x: targetX - warehouseSpace * 0; y: -warehouseHallwayY; teta: -90; advance: NormalAdvance; kind: bpkTarget),
      (x: targetX - warehouseSpace * 1; y: -warehouseHallwayY; teta: -90; advance: NormalAdvance; kind: bpkTarget),
      (x: targetX - warehouseSpace * 2; y: -warehouseHallwayY; teta: -90; advance: NormalAdvance; kind: bpkTarget),
      (x: targetX - warehouseSpace * 3; y: -warehouseHallwayY; teta: -90; advance: NormalAdvance; kind: bpkTarget),
      (x: targetX - warehouseSpace * 4; y: -warehouseHallwayY; teta: -90; advance: NormalAdvance; kind: bpkTarget),

      (x: sourceX; y: 0.08; teta: 0; advance: NormalAdvanceX; kind: bpkMachine),
      (x: 0; y: 0.09; teta: 179; advance: 0; kind: bpkMachine),  // x,advance dinamically set in BuildStateMachine
      (x: sourceX; y: -0.09; teta: 0; advance: NormalAdvanceX; kind: bpkMachine),
      (x: 0; y: -0.09; teta: 179; advance: 0; kind: bpkMachine), // x,advance dinamically set in BuildStateMachine

      (x: 0; y: 0.08; teta: 0; advance: 0; kind: bpkMachine),    // x,advance dinamically set in BuildStateMachine
      (x: targetX; y: 0.09; teta: 179; advance: NormalAdvanceX; kind: bpkMachine),
      (x: 0; y: -0.09; teta: 0; advance: 0; kind: bpkMachine),   // x,advance dinamically set in BuildStateMachine
      (x: targetX; y: -0.09; teta: 179; advance: NormalAdvanceX; kind: bpkMachine),

      // starting position
      (x: sourceX; y: -0.57; teta: 90; advance: NormalAdvance; kind: bpkStart)
  );

procedure BuildStateMachine(safe, test: boolean);
function doGoToXY(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
function doPosControl(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
function doStop(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
procedure AddHalfRun(var start, dest: TBoxPlace);
procedure ControlRobot(var v, vn, w: double; var RobotState: TRobotState);
procedure BuildPathStateMachine;
procedure ResetStateMachine;

implementation

uses microtimer, utils, math, laserLoc, CameraUDP;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//     STATE MACHINE EXECUTION      //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

{------------------------------------------------------------------------------
       doGoToXY
------------------------------------------------------------------------------}

function doGoToXY(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var error_teta, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, max_speed, d: double;
begin
  TranslateAndRotate(rpx, rpy, RobotState.x, RobotState.y, -action.x, -action.y, -action.path_teta);
  rpteta := DiffAngle(RobotState.teta, action.path_teta);
  error_teta := DiffAngle(RobotState.teta, action.teta);

  debug(format('rpx: %g', [rpx]));

  //ramp_slope := 2; // approach slope in m/s per meter

  d := Dist(rpx, rpy);

  // limit speed when approaching the end of the path
  (*  max_speed := ramp_slope * d + 0.15;
  if speed > max_speed then
     speed := max_speed;
  // limit acceleration
  max_speed := Dist(RobotState.last_v, RobotState.last_vn) + 1 * 0.025;
  if speed > max_speed then
     speed := max_speed; *)

  vx := - action.speed * Sign(rpx);
  vy := -1 * rpy;
  w := -0.5 * error_teta;

  if abs(error_teta) > 0.2 then begin
     w := -1.9 * error_teta;
  end;

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);

  result := (rpx >= -0.03);
  //result := (d < 0.02) and (error_teta < 0.05);
end;

{------------------------------------------------------------------------------
       doPosControl
------------------------------------------------------------------------------}

function doPosControl(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var error_teta, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, max_speed, d: double;
begin
  TranslateAndRotate(rpx, rpy, RobotState.x, RobotState.y, -action.x, -action.y, -action.path_teta);
  rpteta := DiffAngle(RobotState.teta, action.path_teta);
  error_teta := DiffAngle(RobotState.teta, action.teta);

  d := Dist(rpx, rpy);

  vx := -2 * rpx;
  vy := -2 * rpy;
  w := -3.5 * error_teta;

  //if abs(error_teta) < rad(10) then
     // w := -1.3 * error_teta;

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);
  result := (d < 0.01 )and( abs(error_teta) < Rad(2));
end;

{------------------------------------------------------------------------------
       doServoSwivel
------------------------------------------------------------------------------}

function doServoSwivel(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
begin
  RobotState.servo1 := action.x;
  RobotState.servo2 := action.y;

  v := 0;
  vn := 0;
  w := 0;

  result := (getMicroTime - CurStepStamp) > (round(action.teta) * 1000);
end;

{------------------------------------------------------------------------------
       doRotateAndGo
------------------------------------------------------------------------------}

function doRotateAndGo(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var error_teta, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, speed, max_speed, d: double;
    wanted_teta, completed, path_length: double;
begin
  TranslateAndRotate(rpx, rpy, RobotState.x, RobotState.y, -action.x, -action.y, -action.path_teta);

  completed := 1 - (-rpx / action.path_length);
  completed := completed * 1.3;

  if completed < 0 then begin
     completed := 0;
  end;

  if completed > 1 then begin
     completed := 1;
  end;

  wanted_teta := NormalizeAngle(action.teta_start + DiffAngle(action.teta, action.teta_start) * completed);

  rpteta := DiffAngle(RobotState.teta, action.path_teta);
  error_teta := DiffAngle(RobotState.teta, wanted_teta);

  // approach slope in m/s per meter
 (* ang := abs(DiffAngle(action.path_teta, action.teta));
  if (ang < Pi / 4) or (ang > 3*Pi/4) then begin
     ramp_slope := 4; // approach slope in m/s per meter
  end else begin
           ramp_slope := 3; // approach slope in m/s per meter
  end; *)
  ramp_slope := 20;

  d := Dist(rpx, rpy);

  speed := action.speed;

  // limit speed when approaching the end of the path
  max_speed := ramp_slope * d + action.final_speed;

  if speed > max_speed then begin
     speed := max_speed;
  end;

  // limit acceleration
  max_speed := Dist(RobotState.last_v, RobotState.last_vn) + 1 * 0.025;

  if speed > max_speed then begin
     speed := max_speed;
  end;

  //vx := -speed * sign(rpx);
  vx := speed;
  vy := -1 * rpy;    //2
  w := -2.3 * error_teta;     //1.5

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);

  result := (rpx >= -0.05);
//  result := (d < 0.02) and (error_teta < 0.25);
end;

{------------------------------------------------------------------------------
       doGoToWarehouse
------------------------------------------------------------------------------}

function doGoToWarehouse(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var error_teta, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, speed, max_speed, d: double;
    claw_error, max_claw_error: double;
begin
  TranslateAndRotate(rpx, rpy, RobotState.x, RobotState.y, -action.x, -action.y, -action.path_teta);
  rpteta := DiffAngle(RobotState.teta, action.path_teta);
  error_teta := DiffAngle(RobotState.teta, action.teta);

  // approach slope in m/s per meter
  ramp_slope := 5; // approach slope in m/s per meter

  d := Dist(rpx, rpy);

  speed := action.speed;
  // limit speed when approaching the end of the path
  (*  max_speed := ramp_slope * d;
  if speed > max_speed then
    speed := max_speed;*)
  // limit acceleration

  max_speed := Dist(RobotState.last_v, RobotState.last_vn) + 1 * 0.025;

  if speed > max_speed then  begin
     speed := max_speed;
  end;

  // 0.30 is the distance from the center of the robot to the end of the claw
  claw_error := abs(rpy + error_teta * clawCenterDistance);
  max_claw_error := 0.04;

  if claw_error > max_claw_error then begin
     max_speed := 0;
  end else begin
      max_speed := cos(claw_error * (Pi / 2) / max_claw_error) * speed;
  end;

  if speed > max_speed then begin
     speed := max_speed;
  end;

  //vx := -speed * sign(rpx);
  vx := speed;
  vy := -1.5 * rpy;        //2.5
  w := -2.5 * error_teta;       //2.5

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);

  result := rpx >= -0.0;

  if result and action.reset_coords then begin
     RobotState.x := action.reset_x;
     RobotState.y := action.reset_y;
  end;

  //result := (d < 0.02) and (error_teta < 0.05);

end;

{------------------------------------------------------------------------------
       doFollowArc
------------------------------------------------------------------------------}

function doFollowArc(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
var x_ini, y_ini, teta_ini: double;
    x_end, y_end, teta_end: double;
    xc, yc: double;
    error_teta, path_arc, ang: double;
    rpx, rpy, rpteta: double;
    vx, vy, ramp_slope, speed, max_speed, d: double;
    wanted_teta, completed, path_length: double;
begin
  (* TODO *)
  (*
    atype := atFollowArc;
    x := px_ini;
    y := py_ini;
    teta := pteta_ini * Pi / 180;
    path_teta := pteta_end * Pi / 180;
    radius := pradius;
    rteta_start := prteta_start * Pi / 180;
    rteta_end := prteta_end * Pi / 180;
    speed = pspeed;
  *)

  x_ini := action.x;
  y_ini := action.y;

  xc := x_ini - cos(action.rteta_start) * action.radius;
  yc := y_ini - sin(action.rteta_start) * action.radius;

  x_end := xc + cos(action.rteta_end) * action.radius;
  y_end := yc + sin(action.rteta_end) * action.radius;

  teta_ini := action.teta;
  teta_end := action.path_teta;
  path_arc := DiffAngle(action.rteta_end, action.rteta_start);

  ang := ATan2(RobotState.y - yc, RobotState.x - xc);
  rpy := sign(path_arc) * (action.radius - Dist(RobotState.y - yc, RobotState.x - xc));
  completed := DiffAngle(ang, action.rteta_start) / path_arc;

  if completed < 0 then begin
     completed := 0;
  end;

  if completed > 1 then  begin
     completed := 1;
  end;

  wanted_teta := NormalizeAngle(teta_ini + DiffAngle(teta_end, teta_ini) * completed);

  rpteta := DiffAngle(RobotState.teta, NormalizeAngle(ang + (Pi/2) * sign(path_arc)));
  error_teta := DiffAngle(RobotState.teta, wanted_teta);

  speed := action.speed;

  // limit acceleration
  max_speed := Dist(RobotState.last_v, RobotState.last_vn) + 1 * 0.025;
  if speed > max_speed then begin
     speed := max_speed;
  end;

  vx := speed;
  vy := -2 * rpy;     //-2
  w := -2.3 * error_teta;  //1.5

  TranslateAndRotate(v, vn, 0, 0, vx, vy, -rpteta);

  result := (completed >= 0.97);
end;

{------------------------------------------------------------------------------
       doStop
------------------------------------------------------------------------------}

function doStop(var action: Taction; var v, vn, w: double; var RobotState: TRobotState): boolean;
begin

  v := 0;
  vn := 0;
  w := 0;

  result := false;

end;

{------------------------------------------------------------------------------
       ControlRobot
------------------------------------------------------------------------------}

procedure ControlRobot(var v, vn, w: double; var RobotState: TRobotState);
var complete: boolean;
begin

  if FHal.CBDebug.checked then begin
    FHal.EditXDebug.text :=  Floattostr(ActionScript[CurStep].x);
    FHal.EditYDebug.text :=  Floattostr(ActionScript[CurStep].y);
    FHal.EditThDebug.text :=  Floattostr(DEG(ActionScript[CurStep].teta));
  end;

  case ActionScript[CurStep].atype of

       atGoToXY: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'GoToXY';
                 end;

          complete := doGoToXY(ActionScript[CurStep], v, vn, w, RobotState);
       end;

       atSwivelServo: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'Swivel';
                 end;

      complete := doServoSwivel(ActionScript[CurStep], v, vn, w, RobotState);
      end;

      atRotateAndGo: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'RotateAndGo';
                 end;

      complete := doRotateAndGo(ActionScript[CurStep], v, vn, w, RobotState);
      end;

      atGoToWarehouse: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'GoToWarehouse';
                 end;

      complete := doGoToWarehouse(ActionScript[CurStep], v, vn, w, RobotState);
      end;

      atFollowArc: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'FollowArc';
                 end;

      complete := doFollowArc(ActionScript[CurStep], v, vn, w, RobotState);
      end;

      atStop: begin

                 if FHal.CBDebug.checked then begin
                    FHal.EActionDebug.text := 'Stop';
                 end;

      complete := doStop(ActionScript[CurStep], v, vn, w, RobotState);
      end;

      atWait: begin
      if (rebuild = 0) then begin
         case FHal.RGMission.ItemIndex of
              0 : PathSelectionMission1();
              1 : PathSelectionMission2();
              2 : PathSelectionMission3();
         end;

      BuildPathStateMachine;
      (*ResetStateMachine();*)

      v := 0;
      vn := 0;
      w := 0;

      complete := true;
      end else begin
          case FHal.RGOperationMode.ItemIndex of
              0 : BuildStateMachine(false, false);
              1 : BuildStateMachine(true, false);
              2 : BuildStateMachine(false, true);
          end;
          ResetStateMachine();
          rebuild:=0;
          end;
      end;

      atCheck: begin

      totalParts:=totalParts+1;

      v := 0;
      vn := 0;
      w := 0;

      if totalParts = 1 then begin
          ve := 1;
      end else begin
          ve:=2;
      end;

      complete := true;

      end;

      atConfirm: begin
          if ((totalParts = 5) (*and (ve = 25)*)) then begin
               FHal.ChangePartColor(PartsScript);
               ve := 5;
               complete := true;
          end else begin
               complete := false;
          end;
      end;

      atStabilize: begin

      count := count + 1;
      v := 0;
      vn := 0;
      w := 0;

      if count > 30000 then begin
         count2 := count2 + 1;

         if count2 > 30000 then begin
            count3 := count3 + 1;
            count2 := 0;
            end;
         count := 0;
      end;

      if count3 > 50 then begin
         complete := True;
         end
         else begin
              complete:=false;
         end;
      end;
  end;

  if complete then begin
     Inc(CurStep);
     CurStepStamp := getMicroTime;
     //FMain.MemoDebug.Lines.Add(Format('%d: %d', [integer(CurStepStamp div 1000000), CurStep]));
  end;

end;

{------------------------------------------------------------------------------
       ResetStateMachine
------------------------------------------------------------------------------}

procedure ResetStateMachine;
begin
  CurStep := 0;
  CurStepStamp := getMicroTime;
end;



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//     STATE MACHINE CREATION       //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//          BASIC ACTIONS           //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

{------------------------------------------------------------------------------
       asAddGoToXY
------------------------------------------------------------------------------}

procedure asAddGotoXY(px, py, pteta, ppath_teta, pspeed: double);
begin
  with ActionScript[ActionScriptCount] do begin
    atype := atGoToXY;
    x := px;
    y := py;
    teta := pteta;
    path_teta := ppath_teta;
    speed := pspeed;
  end;
  Inc(ActionScriptCount);
end;

{------------------------------------------------------------------------------
       asAddSwivelServo
------------------------------------------------------------------------------}

procedure asAddSwivelServo(servoDegree1, servoDegree2, timeout: double);
begin
  with ActionScript[ActionScriptCount] do begin
    atype := atSwivelServo;
    x := servoDegree1;
    y := servoDegree2;
    teta := timeout;
  end;
  Inc(ActionScriptCount);
end;

{------------------------------------------------------------------------------
       asAddRotateAndGo
------------------------------------------------------------------------------}

procedure asAddRotateAndGo(px_start, py_start, pteta_start, px, py, pteta, pspeed, pfinal_speed: double);
begin
  with ActionScript[ActionScriptCount] do begin
    atype := atRotateAndGo;
    x := px;
    y := py;
    teta := pteta * Pi / 180;
    path_teta := ATan2(py - py_start, px - px_start);
    speed := pspeed;
    final_speed := pfinal_speed;

    path_length := Dist(px - px_start, py - py_start);
    teta_start := pteta_start * Pi / 180;
  end;
  Inc(ActionScriptCount);
end;

{------------------------------------------------------------------------------
       asAddGoToWarehouse
------------------------------------------------------------------------------}

procedure asAddGotoWarehouse(px, py, pteta, ppath_teta, preset_x, preset_y, pspeed: double; preset_coords: boolean);
begin
  with ActionScript[ActionScriptCount] do begin
    atype := atGoToWarehouse;
    x := px;
    y := py;
    teta := pteta * Pi / 180;
    path_teta := ppath_teta * Pi / 180;
    speed := pspeed;

    reset_x := preset_x;
    reset_y := preset_y;
    reset_coords := preset_coords;
  end;
  Inc(ActionScriptCount);
end;

{------------------------------------------------------------------------------
       asAddFollowArc
------------------------------------------------------------------------------}

procedure asAddFollowArc(px_ini, py_ini, pteta_ini, pteta_end, pradius, prteta_start, prteta_end, pspeed: double);
begin
  with ActionScript[ActionScriptCount] do begin
    atype := atFollowArc;
    x := px_ini;
    y := py_ini;
    teta := pteta_ini * Pi / 180;
    path_teta := pteta_end * Pi / 180;
    radius := pradius;
    rteta_start := prteta_start * Pi / 180;
    rteta_end := prteta_end * Pi / 180;
    speed := pspeed;
  end;
  Inc(ActionScriptCount);
end;

{------------------------------------------------------------------------------
       asAddStop
------------------------------------------------------------------------------}

procedure asAddStop;
begin
  ActionScript[ActionScriptCount].atype := atStop;
  Inc(ActionScriptCount);
end;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//      INTERMEDIATE ACTIONS        //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

{------------------------------------------------------------------------------
       GrabBox
------------------------------------------------------------------------------}

procedure GrabBox(x_path, y_path, x_ware, y_ware: double);
var teta, reset_x, reset_y: double;
begin
  teta := ATan2(y_ware - y_path, x_ware - x_path) * 180 / Pi;
  reset_x := (x_path + x_ware) / 2;
  reset_y := (y_path + y_ware) / 2;

  asAddGotoWarehouse(x_ware, y_ware, teta, teta, reset_x, reset_y, warehouseApproachSpeed, true);
  asAddSwivelServo(2000, 2000, 500);
  asAddGotoWarehouse(x_path, y_path, teta, deg(NormalizeAngle(Rad(teta) + Pi)), 0, 0, warehouseLeaveSpeed, false);
  //asAddGotoXY(x_path, y_path, teta, Deg(NormalizeAngle(Rad(teta) + Pi)), defaultSpeed);
  //asAddSwivelServo(2000, 2000, 0);
end;

{------------------------------------------------------------------------------
       DropBox
------------------------------------------------------------------------------}

procedure DropBox(x_path, y_path, x_ware, y_ware: double);
var teta, reset_x, reset_y: double;
begin
  teta := ATan2(y_ware - y_path, x_ware - x_path) * 180 / Pi;
  reset_x := (x_path + x_ware) / 2;
  reset_y := (y_path + y_ware) / 2;

  asAddSwivelServo(1000, 1000, 0);
  asAddGotoWarehouse(x_ware, y_ware, teta, teta, reset_x, reset_y, warehouseApproachSpeed, true);
  asAddSwivelServo(0, 0, 750);
  asAddGotoWarehouse(x_path, y_path, teta, Deg(NormalizeAngle(Rad(teta) + Pi)), 0, 0, warehouseLeaveSpeed, false);
end;

{------------------------------------------------------------------------------
       DoRoundCorner
------------------------------------------------------------------------------}

procedure DoRoundCorner(x1, y1, xcorner, ycorner, x2, y2, radius, teta_start, teta_end, speed, final_speed: double);
var v1x, v1y, v1d, v2x, v2y, v2d: double;
    u1x, u1y, u2x, u2y: double;
    i1x, i1y, i2x, i2y: double;
    iteta1, iteta2, total_dist: double;
begin
  teta_start := Rad(teta_start);
  teta_end := Rad(teta_end);

  v1x := x1 - xcorner;
  v1y := y1 - ycorner;
  v1d := Dist(v1x, v1y);
  u1x := v1x / v1d;
  u1y := v1y / v1d;

  v2x := x2 - xcorner;
  v2y := y2 - ycorner;
  v2d := Dist(v2x, v2y);
  u2x := v2x / v2d;
  u2y := v2y / v2d;

  i1x := xcorner + u1x * radius;
  i1y := ycorner + u1y * radius;
  i2x := xcorner + u2x * radius;
  i2y := ycorner + u2y * radius;

  total_dist := v1d + v2d + ((Pi/2) - 2) * radius;

  iteta1 := AvgAngle(teta_start, teta_end, (v1d - radius) / total_dist);
  iteta2 := AvgAngle(teta_start, teta_end, (total_dist - (v2d - radius)) / total_dist);

  if v1d > radius + 1e-3 then
    asAddRotateAndGo(x1, y1, Deg(teta_start), i1x, i1y, Deg(iteta1), speed, speed);
  asAddFollowArc(i1x, i1y, Deg(iteta1), Deg(iteta2), radius, Deg(NormalizeAngle(ATan2(v2y,v2x) + Pi)),
                 Deg(NormalizeAngle(ATan2(v1y,v1x) + Pi)), speed);
  if v2d > radius + 1e-3 then
    asAddRotateAndGo(i2x, i2y, Deg(iteta2), x2, y2, Deg(teta_end), speed, final_speed);
end;

{------------------------------------------------------------------------------
       DoCenterRoundCorner
------------------------------------------------------------------------------}

procedure DoCenterRoundCorner(x1, y1, x2, y2, radius, teta_start, teta_end, speed, final_speed: double);
begin
  if (y1 > 0.25) or (y2 > 0.25) then begin
    DoRoundCorner(x1, y1, max(x1,x2), max(y1,y2), x2, y2, radius, teta_start, teta_end, speed, final_speed);
  end else begin
    DoRoundCorner(x1, y1, min(x1,x2), min(y1,y2), x2, y2, radius, teta_start, teta_end, speed, final_speed);
  end;
end;

{------------------------------------------------------------------------------
       AddHalfRun
------------------------------------------------------------------------------}

procedure AddHalfRun(var start, dest: TBoxPlace);
var teta: double;
begin
  if (start.kind = bpkStart) and (dest.kind = bpkSource) then begin
    if (abs(start.x - dest.x) < 1e-6) or (abs(start.y - dest.y) < 1e-6) then begin
      asAddGotoXY(dest.x, dest.y, rad(dest.teta), ATan2(dest.y-start.y,dest.x-start.x), defaultSpeed);
    end else begin
      DoRoundCorner(start.x, start.y, start.x, dest.y, dest.x, dest.y, 0.15, start.teta, dest.teta, defaultSpeed, defaultFinalSpeed);
    end;
    exit;
  end;

  if ((start.kind = bpkSource) and (dest.kind = bpkTarget))
      or ((dest.kind = bpkSource) and (start.kind = bpkTarget)) then begin
    if start.kind = bpkTarget then teta := 0 else teta := 180;
    DoCenterRoundCorner(start.x, start.y, 0, 0, pathRadius, start.teta, teta, defaultSpeed, defaultSpeed);
    DoCenterRoundCorner(0, 0, dest.x, dest.y, pathRadius, teta, dest.teta, defaultSpeed, defaultFinalSpeed);
    exit;
  end;

  if (start.kind = bpkMachine) and (dest.kind = bpkMachine) then begin
    //asAddGotoXY(dest.x, dest.y, Rad(dest.teta), ATan2(dest.y-start.y,dest.x-start.x), defaultSpeed);
    asAddRotateAndGo(start.x, start.y, start.teta, dest.x, dest.y, dest.teta, defaultSpeed, defaultFinalSpeed);
    exit;
  end;

  DoCenterRoundCorner(start.x, start.y, dest.x, dest.y, pathRadius, start.teta, dest.teta, defaultSpeed, defaultFinalSpeed);
end;

{------------------------------------------------------------------------------
       AddRobotRun
------------------------------------------------------------------------------}

procedure AddRobotRun(last_drop, pick, drop: integer);
var n1, n2, n3: TBoxPlace;
    teta: double;
begin
  n1 := BoxPlaces[last_drop];
  n2 := BoxPlaces[pick];
  n3 := BoxPlaces[drop];

  AddHalfRun(n1, n2);
  teta := Rad(n2.teta);
  GrabBox(n2.x, n2.y, n2.x + cos(teta) * n2.advance, n2.y + sin(teta) * n2.advance);
  AddHalfRun(n2, n3);
  teta := Rad(n3.teta);
  DropBox(n3.x, n3.y, n3.x + cos(teta) * n3.advance, n3.y + sin(teta) * n3.advance);
end;

{------------------------------------------------------------------------------
       BuildPathStateMachine
------------------------------------------------------------------------------}

procedure BuildPathStateMachine;

var i, last_drop, waitForBoxes: integer;
begin

  last_drop := 18;
  case FHal.RGMission.ItemIndex of

      0 : begin
           for i := 0 to High(path_1) do begin
            AddRobotRun(last_drop, path_1[i,0], path_1[i,1]);
            last_drop := path_1[i,1];
           end;
           asAddStop;
          end;

      1 : begin
           for i := 0 to High(path_2) do begin
             AddRobotRun(last_drop, path_2[i,0], path_2[i,1]);
             last_drop := path_2[i,1];
           end;
           asAddStop;
          end;

      2 : begin
           for i := 0 to High(path_3) do begin
            AddRobotRun(last_drop, path_3[i,0], path_3[i,1]);
            last_drop := path_3[i,1];
           end;
           asAddStop;
          end;
  end;
end;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//     STATE MACHINE BUILDING       //
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

procedure BuildStateMachine(safe, test: boolean);
var machine_approach_x, advance: double;
    waitForBoxes: integer;
    n1, n2, n3: TBoxPlace;
begin

  if safe and not test then begin
    // safe parameters
    defaultSpeed := 0.3;
    warehouseApproachSpeed := 0.15;
    pathRadius := 0.4;
    machine_approach_x := 0.0;
  end else if test then begin
    // test parameters
    defaultSpeed := StrToFloat(FHal.EDefaultSpeed.Text);
    warehouseApproachSpeed := StrToFloat(FHal.EApproachSpeed.Text);
    pathRadius := StrToFloat(FHal.EPathRadius.Text);
    machine_approach_x := 0.1;
  end else begin
    // unsafe parameters
    defaultSpeed := 0.4;
    warehouseApproachSpeed := 0.3;
    pathRadius := 0.4;
    machine_approach_x := 0.1;
  end;

  advance := warehouseX - machine_approach_x;

  BoxPlaces[11].x := -machine_approach_x;
  BoxPlaces[13].x := -machine_approach_x;
  BoxPlaces[14].x := machine_approach_x;
  BoxPlaces[16].x := machine_approach_x;

  BoxPlaces[11].advance := advance;
  BoxPlaces[13].advance := advance;
  BoxPlaces[14].advance := advance;
  BoxPlaces[16].advance := advance;

  ActionScriptCount := 0;
 { asAddGotoXY(-1.11, 0.6, pi/2, pi/2, 0.15);
  asAddGotoXY(0, 0.6, pi/2, 0, 0.15);
  asAddGotoXY(0, -0.6, pi/2, -pi/2, 0.15);
  asAddGotoXY(-1.11, -0.6, pi/2, -pi, 0.15);
  asAddStop();
  ResetStateMachine();          }

  if FHal.RGMission.ItemIndex <> 0 then begin

     totalParts := 0;
     ve := 0;

     n1 := BoxPlaces[18];
     n2 := BoxPlaces[0];
     count := 0;
     count2 := 0;
     count3 := 0;
     AddHalfRun(n1, n2);

     defaultSpeed := 0.519;

     (* asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.2);        *)

    FHal.debugBoxColor.text := 'On a Mission!';
    // FHal.Memo1.Append('Tá Ganho!');

    with ActionScript[ActionScriptCount] do begin
         atype := atStabilize;
    end;

    with ActionScript[ActionScriptCount] do begin
         atype := atCheck;
    end;

    Inc(ActionScriptCount);

    n1 := BoxPlaces[0];
    n2 := BoxPlaces[1];

    count := 0;
    count2 := 0;
    asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.6);
    // FHal.Memo1.Append('Desculpem Amigos!');

    with ActionScript[ActionScriptCount] do begin
         atype := atStabilize;
    end;

    with ActionScript[ActionScriptCount] do begin
         atype := atCheck;
    end;

    Inc(ActionScriptCount);

    n1 := BoxPlaces[1];
    n2 := BoxPlaces[2];

    count := 0;
    count2 := 0;
    asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.2);

    with ActionScript[ActionScriptCount] do begin
         atype := atStabilize;
    end;

    with ActionScript[ActionScriptCount] do begin
         atype := atCheck;
    end;

    Inc(ActionScriptCount);

    n1 := BoxPlaces[2];
    n2 := BoxPlaces[3];

    count := 0;
    count2 := 0;
    asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.2);

    with ActionScript[ActionScriptCount] do begin
         atype := atStabilize;

    end;

    with ActionScript[ActionScriptCount] do begin
         atype := atCheck;
    end;

    Inc(ActionScriptCount);

    n1 := BoxPlaces[3];
    n2 := BoxPlaces[4];

    count := 0;
    count2 := 0;
    asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.2);

    with ActionScript[ActionScriptCount] do begin
         atype := atStabilize;
    end;

    with ActionScript[ActionScriptCount] do begin
         atype := atCheck;
    end;

    Inc(ActionScriptCount);

    with ActionScript[ActionScriptCount] do begin
         atype := atConfirm;
    end;

    Inc(ActionScriptCount);

    n1 := BoxPlaces[4];
    n2 := BoxPlaces[0];

    count := 0;
    count2 := 0;
    asAddGotoXY(n2.x, n2.y, rad(n2.teta), ATan2(n2.y-n1.y,n2.x-n1.x), 0.2);

    n1 := BoxPlaces[0];
    n2 := BoxPlaces[18];

    asAddGotoXY(n2.x, 0, rad(n2.teta), ATan2(0,n2.x-n1.x), defaultSpeed);

    with ActionScript[ActionScriptCount] do begin
         atype := atWait;
    end;

    Inc(ActionScriptCount);

    (*BuildPathtateMachine;*)
    (*asAddStop();*)

  end else begin
      BuildPathStateMachine;
  end;

end;


end.

