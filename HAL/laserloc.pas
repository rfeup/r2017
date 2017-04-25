unit laserloc;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ComCtrls, IniPropStorage, ExtCtrls, lNetComponents, lNet, TAGraph,
  TASeries, Rlan, math, LclIntf, epiktimer, utils, CameraUDP, structs, state, hal;

type
  TLaserPoint = record
    d, angle: double;
    x, y: double;
  end;

  TLaserPoints = array of TLaserPoint;

  TPose = record
    x, y, theta, err: double;
  end;

  TOdomVel = record
    Vx, Vy, W: double;
  end;

  TWheelVel = record
    V1, V2, V3: double;
  end;

  { TFLaserLoc }

  TFLaserLoc = class(TForm)
    BLogClear: TButton;
    BOdoSet: TButton;
    BRobotPosSet: TButton;
    BSettingsSet: TButton;
    BLogSave: TButton;
    BResetActionCount: TButton;
    CBShowSize: TCheckBox;
    ChartXY: TChart;
    CBInvertedLaser: TCheckBox;
    CBShowLoc: TCheckBox;
    CBSendLock: TCheckBox;
    CBAccumulation: TCheckBox;
    CBEnableLaserLoc: TCheckBox;
    CSLaserXY: TLineSeries;
    DebugThOdo: TEdit;
    DebugXOdo: TEdit;
    DebugYOdo: TEdit;
    EditDt: TEdit;
    EditLogFile: TEdit;
    EditLaserXOffset: TEdit;
    EditNumAccPoints: TEdit;
    EditSendLockIP: TEdit;
    EditMaxIters: TEdit;
    EditFitError: TEdit;
    EditStepScale: TEdit;
    EditCerr: TEdit;
    EditTime: TEdit;
    EditLaserAngleOffset: TEdit;
    EditRobotThetaSet: TEdit;
    EditRobotX: TEdit;
    EditRobotXSet: TEdit;
    EditRobotY: TEdit;
    EditRobotTheta: TEdit;
    EditRobotYSet: TEdit;
    EditWheelDistance: TEdit;
    EditWheelRadius: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    Label10: TLabel;
    Label11: TLabel;
    Label12: TLabel;
    Label13: TLabel;
    Label14: TLabel;
    Label15: TLabel;
    Label16: TLabel;
    Label17: TLabel;
    Label18: TLabel;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    Odo: TTabSheet;
    UDPOdo: TLUDPComponent;
    RGLasers: TRadioGroup;
    UDPCamera: TLUDPComponent;
    UDPSend: TLUDPComponent;
    PageControl: TPageControl;
    PaintBox: TPaintBox;
    TabLaser: TTabSheet;
    TabLaserXY: TTabSheet;
    TabDebug: TTabSheet;
    TabRadial: TTabSheet;
    TabSettings: TTabSheet;
    UDP: TLUDPComponent;
    Memo: TMemo;
    Chart: TChart;
    CSLaser: TLineSeries;
    procedure BLogClearClick(Sender: TObject);
    procedure BLogSaveClick(Sender: TObject);
    procedure BOdoSetClick(Sender: TObject);
    procedure BResetActionCountClick(Sender: TObject);
    procedure BRobotPosSetClick(Sender: TObject);
    procedure BSettingsSetClick(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure UDPCameraReceive(aSocket: TLSocket);
    procedure UDPOdoError(const msg: string; aSocket: TLSocket);
    procedure UDPOdoReceive(aSocket: TLSocket);
    procedure UDPReceive(aSocket: TLSocket);
  private
    procedure ProcessAccumulation;
    { private declarations }
  public
    NetInBuf: TUDPBuffer;
    LaserPoints, TmpLaserPoints: TLaserPoints;
    NumAccPoints: integer;
    tcount: integer;
    RealRobotPose, OdoRobotPose: TPose;
    odo1, odo2, RealRobotV, RealRobotW, dt: double;

    PerformanceCounter: int64;
    PerformanceFrequency: int64;

    LogFile: TStringList;
    procedure ReceiveWrMessage(mtype: integer);
    procedure ReceiveCameraMessage(MessaC:string);
    procedure SendVelServMessage(mtype: integer);
    //procedure SendLoc(ToIP: string; port: integer; Loc: TPose);
  end;

var
  FLaserLoc: TFLaserLoc;

  laserOffsetX: double;
  laserFirstIdx, laserLastIdx: integer;
  LaserAngleOffset: double;
  LaserAngleK1, LaserAngleK2: double;
  MaxIters: integer;
  StepScale: double;
  NetInBuf, NetOutBuf: TUDPBuffer;

  NetInBufOdo: TUDPBuffer;
  Messa: string;
  Wr: TWheelVel;
  init: integer;

  DeltaT, WheelDist, WheelRadius: double;

  Vodo:TOdomVel;
  Vwheel:TWheelVel;
    totalParts: integer;
  validate, repeatScan: boolean;

  Pose_loc:TPose;

  ET: TEpikTimer;

  GeMeCount, GeMeIdx: integer;

  NetInBufC: TUDPBuffer;
  MessaC : string;

procedure debug(mess: string);


implementation

uses paint;

{ TFLaserLoc }

procedure TFLaserLoc.ReceiveCameraMessage(MessaC:String);
var ii,NumberBytes: integer;
    dd: word;
begin
    ClearUDPBuffer(NetInBufC);
    NumberBytes := length(MessaC);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBufC.MessSize := NumberBytes;
    NetInBufC.ReadDisp := 0;
    move(MessaC[1], NetInBufC.data[0], NumberBytes);
    if chr(NetGetByte(NetInBufC)) <> '1' then
       exit;
    if chr(NetGetByte(NetInBufC)) <> '2' then
       exit;
    if chr(NetGetByte(NetInBufC)) <> '3' then
       exit;
    if chr(NetGetByte(NetInBufC)) <> '4' then
       exit;
    if chr(NetGetByte(NetInBufC)) <> '5' then
       exit;


    MsgMachineState[1] := NetGetString(NetInBufC);
    MsgMachineState[2] := NetGetString(NetInBufC);
    MsgMachineState[3] := NetGetString(NetInBufC);
    MsgMachineState[4] := NetGetString(NetInBufC);
    MsgMachineState[5] := NetGetString(NetInBufC);

end;

procedure debug(mess: string);
begin
  with FLaserLoc do begin
    Memo.Lines.Add(mess);
    while Memo.Lines.Count > 500 do begin
      Memo.Lines.Delete(0);
    end;
  end;
end;

function NormalAng(ang: double) : double;
begin

  while (ang <-pi) or (ang>pi) do begin
    if(ang > pi) then begin
      ang:=ang-2*pi;
    end;

    if(ang < -pi) then begin
      ang:=ang+2*pi;
    end;
  end;

  result:=ang;
end;

procedure TFLaserLoc.ReceiveWrMessage(mtype: integer);
var ii,NumberBytes: integer;
    dd: word;
begin
    UDPOdo.GetMessage(Messa);

    ClearUDPBuffer(NetInBufOdo);
    NumberBytes := length(Messa);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBufOdo.MessSize := NumberBytes;
    NetInBufOdo.ReadDisp := 0;
    move(Messa[1], NetInBufOdo.data[0], NumberBytes);
    if chr(NetGetByte(NetInBufOdo)) <> 'V' then
       exit;
    if chr(NetGetByte(NetInBufOdo)) <> 'W' then
       exit;
    if chr(NetGetByte(NetInBufOdo)) <> '3' then
       exit;

    Wr.V1 := NetGetShort(NetInBufOdo)/1000;
    Wr.V2 := NetGetShort(NetInBufOdo)/1000;
    Wr.V3 := NetGetShort(NetInBufOdo)/1000;
end;

procedure TFLaserLoc.SendVelServMessage(mtype: integer);
var i: integer;
    ld: integer;
begin
    ClearUDPBuffer(NetOutBuf);

    NetPutByte(NetOutBuf, ord('V'));
    NetPutByte(NetOutBuf, ord('N'));
    NetPutByte(NetOutBuf, ord('W'));
    NetPutByte(NetOutBuf, ord('S'));

    ld := round(RobotState.v*1000);
    NetPutInt(NetOutBuf, ld);

    ld := round(RobotState.vn*1000);
    NetPutInt(NetOutBuf, ld);

    ld := round(RobotState.W*1000);
    NetPutInt(NetOutBuf, ld);

    ld := round(RobotState.servo1);//servo_right*1000;
    NetPutInt(NetOutBuf, ld);

    ld := round(RobotState.servo2); //servo_left*1000;
    NetPutInt(NetOutBuf, ld);

    UDPSend.Send(NetOutBuf.data, NetOutBuf.MessSize, '127.0.0.1:9006');     //9006
end;

procedure TFLaserLoc.FormShow(Sender: TObject);
begin
  FPaint.show;
  BSettingsSetClick(Sender);
  //UDP.Connect('127.0.0.1',9876)
  UDP.Listen(9876);
  UDPOdo.Connect('127.0.0.1',9001);
  UDPOdo.Listen(9001);
  UDPSend.Connect('127.0.0.1', 9006);
  UDPCamera.Connect('127.0.0.1',9020);
  UDPCamera.Listen(9020);

  BuildStateMachine(true, false);

end;

procedure TFLaserLoc.UDPCameraReceive(aSocket: TLSocket);
var messaC:String;
begin
    UDPCamera.GetMessage(MessaC);

    ReceiveCameraMessage(MessaC);

    UDPCameraReceivePart(messaC);
end;

procedure TFLaserLoc.UDPOdoError(const msg: string; aSocket: TLSocket);
begin

end;

procedure TFLaserLoc.UDPOdoReceive(aSocket: TLSocket);
{Var
dx,dy,dth : double;
Pose_ant : TPose;
Path_test : Taction;
 }
var dx,dy,dth : double;
    Pose_ant : TPose;
    halfTeta, sqrt3, d, r: double;
    v_odo, vn_odo, w_odo: double;
    v, vn, w : double;

begin
  ReceiveWrMessage(0);

  Pose_ant := Pose_loc;

  d := WheelDist;
  r := WheelRadius;
  sqrt3 := sqrt(3);
  v_odo := r / sqrt3 * (Wr.V1 - Wr.V2);
  vn_odo := r / 3 * (Wr.V2 - 2 * Wr.V3 + Wr.V1) ;
  w_odo := -r / (3 * d) * (Wr.V1 + Wr.V2 + Wr.V3) ;

  halfTeta := NormalizeAngle(Pose_loc.theta + w_odo * DeltaT * 0.5);

  Pose_loc.x := Pose_loc.x + (cos(halfTeta) * v_odo + sin(halfTeta) * vn_odo) * DeltaT;
  Pose_loc.y := Pose_loc.y + (-sin(halfTeta) * v_odo + cos(halfTeta) * vn_odo) * DeltaT;
  Pose_loc.theta := NormalizeAngle(Pose_loc.theta + w_odo * DeltaT);

  {Vwheel.V3:=Wr.V1*WheelRadius;
  Vwheel.V1:=Wr.V2*WheelRadius;
  Vwheel.V2:=Wr.V3*WheelRadius;

  dx:=0;
  dy:=0;
  dth:=0;

  Pose_ant := Pose_loc;

  Vodo.Vx:= (((sqrt(3)*cos(Pose_ant.theta))/3)-sin(Pose_ant.theta)/3)*Vwheel.v1 + (2*sin(Pose_ant.theta)/3)*Vwheel.v2 + (((-sqrt(3)*cos(Pose_ant.theta))/3)-sin(Pose_ant.theta)/3)*Vwheel.v3;
  Vodo.Vy:= (((sqrt(3)*sin(Pose_ant.theta))/3)+cos(Pose_ant.theta)/3)*Vwheel.v1 - (2*cos(Pose_ant.theta)/3)*Vwheel.v2 + (((-sqrt(3)*sin(Pose_ant.theta))/3)+cos(Pose_ant.theta)/3)*Vwheel.v3;
  Vodo.W:=  (1/(3*WheelDist))*Vwheel.v1 + (1/(3*WheelDist))*Vwheel.v2 + (1/(3*WheelDist))*Vwheel.v3;

  dx:=Vodo.Vx*DeltaT;
  dy:=Vodo.Vy*DeltaT;
  dth:=Vodo.W*DeltaT;

  Pose_loc.x:= dx+ Pose_ant.x;
  Pose_loc.y:= dy+ Pose_ant.y;
  Pose_loc.theta:=NormalAng(dth+ Pose_ant.theta);
          }
  RobotState.x := Pose_loc.x;
  RobotState.y := Pose_loc.y;
  RobotState.teta := Pose_loc.theta;

  DebugXOdo.Text:= FloatToStr(Pose_loc.x);
  DebugYOdo.Text:= FloatToStr(Pose_loc.y);
  DebugThOdo.Text:= FloatToStr(Pose_loc.theta);

  RobotState.last_v := RobotState.v;
  RobotState.last_vn := RobotState.vn;
  RobotState.last_w := RobotState.w;

  v:=0;
  vn:=0;
  w:=0;

  ControlRobot(v, vn, w, RobotState);
  //debug(inttostr(CurStep));

  RobotState.v := v;
  RobotState.vn := vn;
  RobotState.w := w;

  if (CBSendLock.Checked) then
     SendVelServMessage(0);

  {if CBSendLock.Checked then
    SendLoc(EditSendLockIP.Text, SendLockPort, Pose_loc);
   }
end;

procedure TFLaserLoc.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  ET.Stop;
  FPaint.Close;
end;

procedure TFLaserLoc.FormCreate(Sender: TObject);
begin
  ET := TEpikTimer.Create(Application);
  dt := 0.04;
  LogFile := TStringList.Create;
  DeltaT := StrToIntDef(EditDt.Text, 40) / 1000;
  WheelDist := StrToFloatDef(EditWheelDistance.Text, 0.11);
  WheelRadius := StrToFloatDef(EditWheelRadius.Text, 0.033);
  Pose_loc.x := BoxPlaces[18].x;
  Pose_loc.y := BoxPlaces[18].y;
  Pose_loc.theta := BoxPlaces[18].teta;
  RobotState.servo1 := 0;
  RobotState.servo2 := 0;
  Vodo.Vx := 0.0;
  Vodo.Vy := 0.0;
  Vodo.W := 0.0;
  Wr.V1:=0.0;
  Wr.V2:=0.0;
  Wr.V3:=0.0;
  Vwheel.V1:= 0.0;
  Vwheel.V2:= 0.0;
  Vwheel.V3:= 0.0;
end;

procedure TFLaserLoc.FormDestroy(Sender: TObject);
begin
  LogFile.Free;
  //ET.free;
end;

procedure TFLaserLoc.BSettingsSetClick(Sender: TObject);
var i: integer;
begin
  LaserAngleOffset := degtorad(StrToFloatDef(EditLaserAngleOffset.Text, 0));
  laserOffsetX := StrToFloat(EditLaserXOffset.text);
  MaxIters := StrToIntDef(EditMaxIters.Text, 10);
  StepScale := StrToFloatDef(EditStepScale.Text, 1e-2);
  c_err := StrToFloatDef(EditCerr.Text, 100);

  NumAccPoints := StrToIntDef(EditNumAccPoints.Text, 100);
  SetLength(LaserPoints, NumAccPoints);
  for i := 0 to Length(LaserPoints) - 1 do begin
    with LaserPoints[i] do  begin
      d := 0;
      x := 0;
      y := 0;
    end;
  end;


  with FPaint do
    FPaint. CalcGradMap(GradXMap, GradYMap, DistMap);
end;

procedure TFLaserLoc.BRobotPosSetClick(Sender: TObject);
begin
  Pose_loc.x := StrToFloat(EditRobotXSet.Text);
  Pose_loc.y := StrToFloat(EditRobotYSet.Text);
  Pose_loc.theta := degtorad(StrToFloat(EditRobotThetaSet.Text));
  RobotState.x := Pose_loc.x;
  RobotState.y := Pose_loc.y;
  RobotState.teta := Pose_loc.theta;
end;

procedure TFLaserLoc.BLogSaveClick(Sender: TObject);
begin
  if FileExistsUTF8(EditLogFile.Text) then DeleteFileUTF8(EditLogFile.Text);
  LogFile.SaveToFile(EditLogFile.Text);
end;

procedure TFLaserLoc.BOdoSetClick(Sender: TObject);
begin
  DeltaT := StrToIntDef(EditDt.Text, 40) / 1000;
  WheelDist := StrToFloatDef(EditWheelDistance.Text, 0.1);
  WheelRadius := StrToFloatDef(EditWheelRadius.Text, 0.033);
end;

procedure TFLaserLoc.BResetActionCountClick(Sender: TObject);
begin
  ResetStateMachine();
end;

procedure TFLaserLoc.BLogClearClick(Sender: TObject);
begin
  LogFile.Clear;
  OdoRobotPose := RealRobotPose;
end;

procedure TFLaserLoc.UDPReceive(aSocket: TLSocket);
var data: string;
    NumberBytes: integer;
    id: char;
    i: integer;
    dx, dy: double;
    LaserInv: double;
    delta: int64;
begin
  //Memo.Clear;
  UDP.GetMessage(data);

  ClearUDPBuffer(NetInBuf);
  NumberBytes := length(data);

  if (NumberBytes = 0) or (NumberBytes >= UDPBufSize) then
    exit;

  NetInBuf.MessSize := NumberBytes;
  NetInBuf.ReadDisp := 0;
  move(data[1], NetInBuf.data[0], NumberBytes);

  if RGLasers.ItemIndex = 2 then begin
    ProcessAccumulation();
    exit;
  end;

  ET.Clear;
  ET.Start;

  if RGLasers.ItemIndex = 0 then begin
    // we only expect packets from Hokuyo
    if chr(NetGetByte(NetInBuf)) <> 'H' then
      exit;
    if chr(NetGetByte(NetInBuf)) <> 'A' then
      exit;
    if chr(NetGetByte(NetInBuf)) <> '1' then
      exit;

    laserFirstIdx := 44;
    laserLastIdx := 725;
    SetLength(LaserPoints, 768);
    LaserAngleK1 := 270 / (3*256);
    LaserAngleK2 := -135;

  end else if RGLasers.ItemIndex = 1 then begin
    // we only expect packets from Neato
    if chr(NetGetByte(NetInBuf)) <> 'N' then
      exit;
    if chr(NetGetByte(NetInBuf)) <> 'A' then
      exit;
    if chr(NetGetByte(NetInBuf)) <> '1' then
      exit;

    laserFirstIdx := 0;
    laserLastIdx := 359;
    SetLength(LaserPoints, 360);
    LaserAngleK1 := 1;
    LaserAngleK2 := 0.5;
  end;

  if CBInvertedLaser.Checked then LaserInv := -1
  else LaserInv := 1;

  for i := laserFirstIdx to laserLastIdx do begin
    with LaserPoints[i] do begin
      d := netgetword(NetInBuf) / 1000;
      //angle := LaserInv * degtorad(270 * i / (3*256) - 135);// + LaserAngleOffset;
      angle := LaserInv * degtorad(LaserAngleK1 * i + LaserAngleK2) + LaserAngleOffset;
      x := d * cos(angle);
      y := d * sin(angle);
      //CSLaserXY.AddXY(x, y);
    end;
  end;

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text := format('%d', [delta]);
  ET.Clear;
  ET.Start;

  RobotPose := Pose_loc;
  laserOffsetX := StrToFloatDef(EditLaserXOffset.text, 0.05925);// - 0.015;
  dx := laserOffsetX * cos(RobotPose.theta);
  dy := laserOffsetX * sin(RobotPose.theta);
  RobotPose.x := RobotPose.x + dx;
  RobotPose.y := RobotPose.y + dy;

  for i := 0 to MaxIters do begin
    FPaint.IterLaser(RobotPose, LaserPoints, laserFirstIdx, laserLastIdx, StepScale);
  end;
  EditFitError.Text := format('%.5g', [RobotPose.err]);

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text := EditTime.Text + format(',pf: %d', [delta]);

  ET.Clear;
  ET.Start;

  if CBEnableLaserLoc.Checked then begin
    laserOffsetX := -StrToFloatDef(EditLaserXOffset.text, 0.05925);// - 0.015;
    dx := laserOffsetX * cos(RobotPose.theta);
    dy := laserOffsetX * sin(RobotPose.theta);
    Pose_loc.x := RobotPose.x - dx;
    Pose_loc.y := RobotPose.y - dy;

    Pose_loc.theta:= RobotPose.theta;     //100% laser
    Pose_loc.err:= RobotPose.err;

    RobotState.x := Pose_loc.x;
    RobotState.y := Pose_loc.y;
    RobotState.teta := Pose_loc.theta;
  end;


  //EditRobotX.Text := format('%3f, %3f, %5f', [RobotPose.x - dx, RobotPose.y - dy, radtodeg(RobotPose.theta)]);

  EditRobotX.Text := format('%5f', [Pose_loc.x]);
  EditRobotY.Text := format('%5f', [Pose_loc.y ]);
  EditRobotTheta.Text := format('%3f', [radtodeg(Pose_loc.theta)]);

  if PageControl.ActivePage = TabLaser then begin
    CSLaser.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaser.AddXY(i, LaserPoints[i].d);
      //Memo.Lines.add(inttostr(d));
    end;
  end else if PageControl.ActivePage = TabLaserXY then begin
    CSLaserXY.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaserXY.AddXY(LaserPoints[i].x, LaserPoints[i].y);
    end;
  end;

  if CBShowLoc.Checked then FPaint.FormPaint(FPaint);

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text :=  EditTime.Text + format(', %d us', [delta]);
  ET.Stop;
end;

procedure TFLaserLoc.ProcessAccumulation;
var id: char;
    i, n: integer;
    dx, dy, dtheta: double;
    LaserInv: double;
    delta: int64;
begin
  SetLength(LaserPoints, NumAccPoints);

  ET.Clear;
  ET.Start;

  // we only expect packets from Generic Measures
  if chr(NetGetByte(NetInBuf)) <> 'G' then
    exit;
  if chr(NetGetByte(NetInBuf)) <> 'A' then
    exit;
  if chr(NetGetByte(NetInBuf)) <> '1' then
    exit;

  laserLastIdx := NetGetWord(NetInBuf) - 1;// Read number of points
  laserFirstIdx := 0;

  SetLength(TmpLaserPoints, laserLastIdx + 1);

  if CBInvertedLaser.Checked then LaserInv := -1
  else LaserInv := 1;

  for i := 0 to laserLastIdx do begin
    with TmpLaserPoints[i] do begin
      x := NetGetInt(NetInBuf) / 1000;
      y := NetGetInt(NetInBuf) / 1000;
      d := sqrt(sqr(x) + sqr(y));
      angle := atan2(y, x);
      //CSLaserXY.AddXY(x, y);
    end;
  end;

  if CBAccumulation.Checked then begin
    tcount := NetGetWord(NetInBuf); // Get time
    odo1 := NetGetFloat(NetInBuf); // Get odo1
    odo2 := NetGetFloat(NetInBuf); // Get odo2
    RealRobotV := 4.36e-4 * (odo1 + odo2)/dt;
    RealRobotW := 3.9e-4 * (odo2 - odo1)/(0.1255 * dt);
    //RealRobotV := NetGetFloat(NetInBuf); // Get speed
    //RealRobotW := NetGetFloat(NetInBuf); // Get angular speed

    RealRobotPose.x := NetGetFloat(NetInBuf); // Get Robot x
    RealRobotPose.y := NetGetFloat(NetInBuf); // Get Robot y
    RealRobotPose.theta := NetGetFloat(NetInBuf); // Get Robot theta

    //RobotPose := RealRobotPose;
    RobotPose.x := RobotPose.x + dt * RealRobotV * cos(RobotPose.theta);
    RobotPose.y := RobotPose.y + dt * RealRobotV * sin(RobotPose.theta);
    RobotPose.theta := RobotPose.theta + dt * RealRobotW;

    with OdoRobotPose do begin
      x := x + dt * RealRobotV * cos(theta);
      y := y + dt * RealRobotV * sin(theta);
      theta := theta + dt * RealRobotW;
      //Memo.Lines.add(format('%.4g %.4g %.4g',[x, y, theta]));
      //if Memo.Lines.Count > 10 then memo.lines.Delete(0);
    end;

    // Bring old points to current position using odometry
    for i := 0 to Length(LaserPoints) - 1 do begin
      with LaserPoints[i] do begin
        dtheta := - dt * RealRobotW;
        dx := - dt * RealRobotV;
        dy := 0;
        //TranslateAndRotate(x, y, x, y, dx, dy, dtheta);
        RotateAndTranslate(x, y, x, y, dx, dy, dtheta);
        angle := angle + dtheta;
      end;
    end;

    // insert the new points
    // make room
    for i := Length(TmpLaserPoints) to Length(LaserPoints) - 1 do begin
      LaserPoints[i - Length(TmpLaserPoints)] := LaserPoints[i];
    end;
    // insert
    for i := 0 to Length(TmpLaserPoints) - 1 do begin
      //LaserPoints[(Length(LaserPoints) - 1) - Length(TmpLaserPoints) + i] := TmpLaserPoints[i];
      LaserPoints[(Length(LaserPoints) - 1) - i] := TmpLaserPoints[i];
    end;

  end else begin // Normal processig without accumulation
    LaserPoints := TmpLaserPoints;
  end;

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text := format('%d', [delta]);

  for i := 0 to MaxIters do begin
    FPaint.IterLaser(RobotPose, LaserPoints, 0, Length(LaserPoints) - 1, StepScale);
  end;
  EditFitError.Text := format('%.5g', [RobotPose.err]);

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text := EditTime.Text + format(', %d', [delta]);

  //laserOffsetX := StrToFloatDef(EditLaserXOffset.text, 0);// 0.1;// - 0.015;
  dx := laserOffsetX * cos(RobotPose.theta);
  dy := laserOffsetX * sin(RobotPose.theta);

 // if CBSendLock.Checked then SendLoc(EditSendLockIP.Text, SendLockPort, RobotPose);

  EditRobotX.Text := format('%.3g', [RobotPose.x - dx]);
  EditRobotY.Text := format('%.3g', [RobotPose.y - dy]);
  EditRobotTheta.Text := format('%.5g', [radtodeg(RobotPose.theta)]);

  if PageControl.ActivePage = TabLaser then begin
    CSLaser.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaser.AddXY(i, LaserPoints[i].d);
      //Memo.Lines.add(inttostr(d));
    end;
  end else if PageControl.ActivePage = TabLaserXY then begin
    CSLaserXY.Clear;
    for i := laserFirstIdx to laserLastIdx do begin
      CSLaserXY.AddXY(LaserPoints[i].x, LaserPoints[i].y);
    end;
  end;

  if CBShowLoc.Checked then FPaint.FormPaint(FPaint);

  delta := round(1e6 * ET.Elapsed); // store the elapsed
  EditTime.Text :=  EditTime.Text + format(', %d us', [delta]);
  ET.Stop;

  if CBAccumulation.Checked then begin
    LogFile.Add(Format('%d %g %g  %g %g %g  %g %g %g  %g %g %g',
                       [tcount, RealRobotV, RealRobotW,
                        RealRobotPose.x, RealRobotPose.y, RealRobotPose.theta,
                        RobotPose.x, RobotPose.y, RobotPose.theta,
                        OdoRobotPose.x, OdoRobotPose.y, OdoRobotPose.theta
                       ]));
  end;

end;


{
procedure TFLaserLoc.SendLoc(ToIP: string; port: integer; Loc: TPose);
var i, start: integer;
begin
    //if not UDP.Connected then exit;

    ClearUDPBuffer(NetOutBuf);
    // tag this packet as a Localization Event
    NetPutByte(NetOutBuf, ord('L'));
    NetPutByte(NetOutBuf, ord('O'));
    NetPutByte(NetOutBuf, ord('C'));
    NetPutByte(NetOutBuf, ord('0'));

    NetPutFloat(NetOutBuf, Loc.x);
    NetPutFloat(NetOutBuf, Loc.y);
    NetPutFloat(NetOutBuf, Loc.theta);

    UDPSend.Send(NetOutBuf.data, NetOutBuf.MessSize, ToIP);
end;
     }
initialization
  {$I laserloc.lrs}

end.
