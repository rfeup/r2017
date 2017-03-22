unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, TAGraph, TASeries, Forms, Controls, Graphics,
  Dialogs, Math, StdCtrls, LclIntf, Grids, ComCtrls, Spin, dynmatrix, dynmatrixutils,
  lNetComponents, types, TACustomSeries, DistMap, bgrabitmap, BGRABitmapTypes, Bitmap_Grid, BinHeap, astarlib,
  PathFollowing, lNet, strutils, rlan, Utils;

type

  { TForm1 }

  TForm1 = class(TForm)
    AssignVal: TButton;
    AstarTab: TTabSheet;
    Button1: TButton;
    Label58: TLabel;
    Label59: TLabel;
    Label60: TLabel;
    UDPTask: TLUDPComponent;
    Y_debug: TEdit;
    V_debug: TEdit;
    Vn_debug: TEdit;
    X_debug: TEdit;
    W_debug: TEdit;
    IP_R2: TEdit;
    Label47: TLabel;
    Label50: TLabel;
    Label51: TLabel;
    Label52: TLabel;
    Label53: TLabel;
    Label54: TLabel;
    Label55: TLabel;
    Label56: TLabel;
    Label57: TLabel;
    Port_R2: TEdit;
    RealRobot: TCheckBox;
    DefMach: TButton;
    Calculate: TButton;
    Connect: TButton;
    D: TFloatSpinEdit;
    Discen: TEdit;
    Disconnect: TButton;
    Draw_map: TButton;
    RobNum: TEdit;
    Robot2X: TEdit;
    Robot2Y: TEdit;
    UDPRob: TLUDPComponent;
    Memo1: TMemo;
    Grid_repSeries2: TLineSeries;
    Grid_repSeries1: TLineSeries;
    Height_map: TEdit;
    Height_obs: TEdit;
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
    Label19: TLabel;
    Label2: TLabel;
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label25: TLabel;
    Label26: TLabel;
    Label27: TLabel;
    Label28: TLabel;
    Label29: TLabel;
    Label3: TLabel;
    Label30: TLabel;
    Label31: TLabel;
    Label32: TLabel;
    Label33: TLabel;
    Label34: TLabel;
    Label35: TLabel;
    Label36: TLabel;
    Label37: TLabel;
    Label38: TLabel;
    Label39: TLabel;
    Label4: TLabel;
    Label40: TLabel;
    Label41: TLabel;
    Label42: TLabel;
    Label43: TLabel;
    Label44: TLabel;
    Label45: TLabel;
    Label46: TLabel;
    Label48: TLabel;
    Label49: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    MemoAstar: TMemo;
    PageControl1: TPageControl;
    Robot: TEdit;
    SimTwoUDPR2: TLUDPComponent;
    SimTwoUDP: TLUDPComponent;
    Grid_rep: TChart;
    Grid_repSeries: TLineSeries;
    Size_grid: TEdit;
    Start: TTabSheet;
    TabSheet1: TTabSheet;
    Width_map: TEdit;
    IP_R1: TEdit;
    Port_R1: TEdit;
    Width_obs: TEdit;
    TH_debug: TEdit;
    x_goal: TEdit;
    x_ini: TEdit;
    X_A_M1: TEdit;
    Y_A_M3: TEdit;
    X_M3: TEdit;
    Y_M3: TEdit;
    X_A_M4: TEdit;
    Y_A_M4: TEdit;
    X_M4: TEdit;
    Y_M4: TEdit;
    X_A_M5: TEdit;
    Y_A_M5: TEdit;
    X_M5: TEdit;
    Y_A_M1: TEdit;
    Y_M5: TEdit;
    X_A_M6: TEdit;
    Y_A_M6: TEdit;
    X_M6: TEdit;
    Y_M6: TEdit;
    X_A_M7: TEdit;
    Y_A_M7: TEdit;
    X_M7: TEdit;
    Y_M7: TEdit;
    x_ini29: TEdit;
    X_M1: TEdit;
    X_M8: TEdit;
    Y_M8: TEdit;
    Y_A_M8: TEdit;
    X_A_M8: TEdit;
    X_A_M9: TEdit;
    Y_A_M9: TEdit;
    X_A_M10: TEdit;
    X_A_M11: TEdit;
    X_A_M12: TEdit;
    Y_A_M12: TEdit;
    Y_A_M11: TEdit;
    Y_M1: TEdit;
    Y_A_M10: TEdit;
    X_M10: TEdit;
    X_M11: TEdit;
    X_M9: TEdit;
    Y_M9: TEdit;
    x_ini44: TEdit;
    Y_M10: TEdit;
    Y_M11: TEdit;
    Y_M12: TEdit;
    X_M12: TEdit;
    X_A_M13: TEdit;
    X_A_M2: TEdit;
    Y_A_M13: TEdit;
    X_M13: TEdit;
    Y_M13: TEdit;
    Y_M14: TEdit;
    X_M14: TEdit;
    Y_A_M14: TEdit;
    X_A_M14: TEdit;
    X_A_M15: TEdit;
    Y_A_M15: TEdit;
    X_A_M16: TEdit;
    Y_A_M2: TEdit;
    X_A_M17: TEdit;
    X_A_M18: TEdit;
    Y_A_M18: TEdit;
    Y_A_M17: TEdit;
    Y_A_M16: TEdit;
    X_M16: TEdit;
    X_M17: TEdit;
    X_M15: TEdit;
    Y_M15: TEdit;
    Y_M16: TEdit;
    Y_M17: TEdit;
    X_M2: TEdit;
    x_ini70: TEdit;
    x_ini71: TEdit;
    X_M18: TEdit;
    Y_M18: TEdit;
    Y_M2: TEdit;
    X_A_M3: TEdit;
    y_goal: TEdit;
    y_ini: TEdit;
    procedure AssignValClick(Sender: TObject);
    procedure Button1Click(Sender: TObject);
    procedure DefMachClick(Sender: TObject);
    procedure CalculateClick(Sender: TObject);
    procedure ConnectClick(Sender: TObject);
    procedure DisconnectClick(Sender: TObject);
    procedure Draw_mapClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SimTwoUDPError(const msg: string; aSocket: TLSocket);
    procedure SimTwoUDPReceive(aSocket: TLSocket);
    procedure SimTwoUDPR2Receive(aSocket: TLSocket);
    procedure UDPRobReceive(aSocket: TLSocket);
    procedure UDPTaskReceive(aSocket: TLSocket);
    procedure Width_mapKeyPress(Sender: TObject; var Key: char);
    procedure Height_mapKeyPress(Sender: TObject; var Key: char);

  private
    { private declarations }
  public
    { public declarations }
  end;

  TMachine = record
    Points : TLine;
    Th : double;
    Sentido : integer;
    MoveX, MoveY : boolean;
  end;

  TPosi = record
    X, Y, Dist: Double;
  end;

  (*TNode = record
      id : integer;
      X, Y : Double;
      f : Double;
      g : Double;
      h : Double;
      Passable : Boolean;
      Visited : Boolean;
      Closed : Boolean;
      Parent : integer;
  end;*)

const
  SCALE = 100;
  VEL_ENTER = 0.1;
  VEL_FAST = 0.1;
  VEL_DEC =0.05;
var

  Form1: TForm1;
  Heig_map: double;
  Wid_map: double;
  Dist_Cen, Wid_obs, Heig_Obs, Sizec, Robo_r :double;
  Bmp_Map : TBGRABitmap;
  //xpto: array[1..3,1..5] of real;
  Map, Obst1, Obst2, Pos_obst, MapExpand:TDMatrix;
  Grid, Dist:TDMatrix;
  N_Livre:integer;
  OBSTACLE : integer = $FF; //real = 255;
  NEAR_OBSTACLE : integer = $7F;  //real = 127;
  //OPEN  : int8 = $04; //real = 100;
  //CLEAR : int8 = $00; //real = 100;
  //CLOSE : int8 = $08; //real = 100;
  //Path : TLine;
  Speed : TRobotControls;
  EnterMach_Flag : boolean = false;
  Machine : array [0..17] of TMachine;
  StateMtoM : Integer;
  Garras_use : Boolean;
  Manga1_State : Integer;
  Set_start : Boolean;
  pos_ini : TMachine;
  servo_left, servo_right, ServoCount : integer;
  R2X, R2Y, R2_r:Double;
  BuildM : Boolean;
  ProjP : Boolean;
  PosAux : TPosi;
  NetOutBuf: TUDPBuffer;
  NetInBuf: TUDPBuffer;
  NetInBufTask: TUDPBuffer;
  Messa, Messa1 : string;
  ActionOrder : Integer;

  LineConfig:Tline;
  TrackFollow:TDataPath;
  FinalPose :TPose;
  ClawPos: Integer;


implementation

{$R *.lfm}

{ TForm1 }


procedure RecievePosMessage(mtype: integer);
var ii,NumberBytes: integer;
    dd: word;
begin
    Form1.UDPRob.GetMessage(Messa);

    ClearUDPBuffer(NetInBuf);
    NumberBytes := length(Messa);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBuf.MessSize := NumberBytes;
    NetInBuf.ReadDisp := 0;
    move(Messa[1], NetInBuf.data[0], NumberBytes);
    if chr(NetGetByte(NetInBuf)) <> 'X' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'Y' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'T' then
       exit;

       X_loc := NetGetInt(NetInBuf)/1000;
       Y_loc := NetGetInt(NetInBuf)/1000;
       Th_loc := NormalAng(NetGetInt(NetInBuf)/1000);

end;

procedure RecieveTaskMessage(mtype: integer);
var ii,NumberBytes: integer;
    dd: word;
begin
    Form1.UDPTask.GetMessage(Messa1);

    ClearUDPBuffer(NetInBufTask);
    NumberBytes := length(Messa1);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBufTask.MessSize := NumberBytes;
    NetInBufTask.ReadDisp := 0;
    move(Messa1[1], NetInBufTask.data[0], NumberBytes);
    if chr(NetGetByte(NetInBufTask)) <> 'A' then
       exit;
    if chr(NetGetByte(NetInBufTask)) <> 'X' then
       exit;
    if chr(NetGetByte(NetInBufTask)) <> 'Y' then
       exit;
    if chr(NetGetByte(NetInBufTask)) <> 'T' then
       exit;
    if chr(NetGetByte(NetInBufTask)) <> 'G' then
       exit;


    ActionOrder:= round(NetGetInt(NetInBufTask)/1000);
    FinalPose.X := NetGetInt(NetInBufTask)/1000;
    FinalPose.Y := NetGetInt(NetInBufTask)/1000;
    FinalPose.Th := NormalAng((NetGetInt(NetInBufTask)/1000)*pi/180);
    ClawPos:= round(NetGetInt(NetInBufTask)/1000);

    form1.MemoAstar.lines.add('X: '+floattostr(FinalPose.X)+' Y: '+floattostr(FinalPose.Y)+' T: '+floattostr(FinalPose.Th)+' A: '+inttostr(ActionOrder));

end;

procedure SendVelServMessage(mtype: integer);
var i: integer;
    ld: integer;
begin
  ClearUDPBuffer(NetOutBuf);

    NetPutByte(NetOutBuf, ord('V'));
    NetPutByte(NetOutBuf, ord('N'));
    NetPutByte(NetOutBuf, ord('W'));
    NetPutByte(NetOutBuf, ord('S'));

    ld := round(Speed.V*1000);
    NetPutInt(NetOutBuf, ld);

    ld := round(Speed.Vn*1000);
    NetPutInt(NetOutBuf, ld);

    ld := round(Speed.W*1000);
    NetPutInt(NetOutBuf, ld);

    ld := servo_right*1000;
    NetPutInt(NetOutBuf, ld);

    ld := servo_left*1000;
    NetPutInt(NetOutBuf, ld);

    Form1.UDPRob.Send(NetOutBuf.data, NetOutBuf.MessSize, Form1.IP_R1.text + ':' +  Form1.Port_R1.text);     //9006
end;


procedure PaintBitmap(bmp : TBGRABitmap; XForm, YForm : integer);
begin

 // bmp.DrawHorizLine(0,round(bmp.Height/2), bmp.Width,BGRA(0,0,255));
//  bmp.DrawVertLine(round(bmp.Width/2),0, bmp.Height,BGRA(0,0,255));

  bmp.Resample(450,300).Draw(Form1.Canvas,XForm,YForm,true);
end;

procedure MakeMap();
begin
  DrawMap(Bmp_Map, Wid_map, Heig_Map, Sizec, Dist_Cen, Wid_obs, Heig_obs);
  BitmapToMatrix(Map, Bmp_Map);
end;

procedure ChangeMap();
begin
   Bmp_Map:= TBGRABitmap.Create(round(Wid_map/Sizec), round(Heig_map/Sizec), BGRA(0,255,0));
   Bmp_map.FillEllipseAntialias(round((R2X*SCALE+Wid_map/2-Sizec/2)/Sizec),round((R2Y*SCALE+Heig_map/2-Sizec/2)/Sizec),R2_r/Sizec,R2_r/Sizec,BGRA(255,0,0));
   MakeMap();
   Bmp_Map.free;
end;

procedure MinkowskySum(var Map1: TDMatrix; Robot_r : Double);
var
i,j : integer;
begin

      Dist:= DistMapMat(Map1,Sizec,Sizec*sqrt(2));

      j:=0;
      while(j < Map1.NumCols) do
      begin
           i:=0;
           while (i < Map1.NumRows) do
           begin
                if Dist.getv(i,j)-1 <= Robot_r then
                begin
                Map1.setv(i,j, OBSTACLE);N_Livre:=N_livre+1;
                end;
                if (Dist.getv(i,j)-1 > Robot_r) and
                   (Dist.getv(i,j)-1 <= Robot_r + Sizec) then begin
                Map1.setv(i,j, 3);
                end;
                if (Dist.getv(i,j)-1 > Robot_r + Sizec) and
                   (Dist.getv(i,j)-1 <= Robot_r + 2*Sizec) then begin
                Map1.setv(i,j, 2);
                end;
                if (Dist.getv(i,j)-1 > Robot_r+2*Sizec) and
                   (Dist.getv(i,j)-1 <= Robot_r + 3*Sizec) then begin
                Map1.setv(i,j,1);
                end;
                i := i + 1;
           end;
           j := j + 1;
      end;

end;

function IsValidPos(Map : TDMatrix; Robot_r,X,Y : Double) : boolean;
begin
     if (round((Y+Heig_map/2-Sizec/2)/Sizec)>Map.NumRows-1) or
        (round((Y+Heig_map/2-Sizec/2)/Sizec)<0) or
        (round((X+Wid_map/2-Sizec/2)/Sizec)>Map.NumCols-1) or
        (round((X+Wid_map/2-Sizec/2)/Sizec)<0) then begin

        Result:=False;

     end else begin

     if Map.Ugetv(round((Y+Heig_map/2-Sizec/2)/Sizec),round((X+Wid_map/2-Sizec/2)/Sizec)) <> OBSTACLE then begin
        Result:=True;
     end else begin
        Result:=False;
     end;

     end;
end;

function IsPosValid(Dist : TDMatrix; Robot_r,X,Y : Double) : boolean;
begin
     if Dist.Ugetv(round((Y+Heig_map/2-Sizec/2)/Sizec),round((X+Wid_map/2-Sizec/2)/Sizec)) > Robot_r then begin
        Result:=True;
     end else begin
        Result:=False;
     end;
end;

function isPassable(var Node : TNode; X,Y: Integer) : Boolean;
begin
        if (X < 0) or (Y < 0) or (X > Map.NumCols-1) or (Y > Map.NumRows-1) then begin
                result := false;
                Exit;
        end;

        if (Map.ugetv(Y,X) = 0) then begin
                Node.Passable := True;
                Node.Near_obstacle := False;
                result := true;
                Exit;
        end else if (Map.ugetv(Y,X) = OBSTACLE) then begin
                Node.Passable := False;
                Node.Near_obstacle := False;
                result := false;
                Exit;
        end else begin
                Node.Passable := True;
                Node.Near_obstacle := True;
                result := true;
        end;

end;

function ProjectPoint(Xf,Yf:Double):TPosi; // XF e YF -> cm
var
    Aux, Aux2 : TPosi;
    Angle, AngleAux : Double;
    i : integer;
begin
    Angle := NormalAng(arctan2(Yf-R2Y*SCALE,Xf-R2X*SCALE));
    Aux.X := R2X*SCALE + cos(Angle)*(R2_r+Robo_r);
    Aux.Y := R2Y*SCALE + sin(Angle)*(R2_r+Robo_r);
    Aux.Dist := 1000000;

    if(IsValidPos(Map,Robo_r,Aux.X,Aux.Y)) then begin
        Result := Aux;
    end else begin
        i:=5;
        while i < 360 do begin
            AngleAux := NormalAng(DiffAngle(Angle,-degtorad(i)));

            Aux2.X := R2X*SCALE + cos(AngleAux)*(R2_r+Robo_r);
            Aux2.Y := R2Y*SCALE + sin(AngleAux)*(R2_r+Robo_r);

            if (IsValidPos(Map,Robo_r,Aux2.X,Aux2.Y)) then begin
                Aux2.Dist := sqrt((Aux2.X-R2X*SCALE)*(Aux2.X-R2X*SCALE)+(Aux2.Y-R2Y*SCALE)*(Aux2.Y-R2Y*SCALE));

                if Aux2.Dist < Aux.Dist then Aux:=Aux2;

            end;
            i := i + 5;
        end;
        Result := Aux;
    end;
end;

procedure EnterMach(Path : TLine; Th : double);
var
   Track : TDataPath;
begin
    Vnomin:=VEL_ENTER;
    Vfinal:=0;

    Path.Xi:=Path.Xi/SCALE;
    Path.Yi:=Path.Yi/SCALE;
    Path.Xf:=Path.Xf/SCALE;
    Path.Yf:=Path.Yf/SCALE;

    Track := ConfigTrack(Path);
    if (Track.X_act < -0.01) and (EnterMach_Flag = False) then begin
        Speed := FollowLine(Track.Xi, Track.X_act, Track.Y_act, Track.delta);
    end else begin
        EnterMach_Flag := True;
        Speed := PosControl(Track.X_act, Track.Y_act, Th, Track.delta);
    end;
end;

procedure DefineMap();
begin
    Wid_map:= strtofloat(Form1.Width_map.Text);
    Heig_map:= strtofloat(Form1.Height_map.Text);
    Sizec := StrtoFloat(Form1.Size_grid.Text);
    Dist_Cen := StrtoFloat(Form1.DisCen.Text);
    Wid_obs := strtofloat(Form1.Width_obs.Text);
    Heig_obs:= strtofloat(Form1.Height_obs.Text);
    Robo_r := StrToFloat(Form1.Robot.Text);

    Bmp_Map:= TBGRABitmap.Create(round(Wid_map/Sizec), round(Heig_map/Sizec), BGRA(0,255,0));

    MakeMap();

    MinkowskySum(Map,StrToFloat(Form1.Robot.Text));
end;

procedure ControlClaw(Pos:integer);
begin
    if (Pos = 0) then begin
        servo_left:=0;
        servo_right:=0;
        ServoCount:= ServoCount + 1;
    end else if (Pos = 1) then begin
        servo_left:=1;
        servo_right:=1;
        ServoCount:= ServoCount + 1;
    end else if(Pos = 2)  then begin
        servo_left:=2;
        servo_right:=2;
        ServoCount:= ServoCount + 1;
    end else begin
        ServoCount := 0;
        Manga1_State := Manga1_State+1;
        EnterMach_Flag:=False;
        StateMtoM:=0;
        Garras_use := False;
    end;
end;

procedure AstarPath(Path : TLine; Th : double);
var
   Startx,Goalx:TNode;
   Heap:TNodeArray;
   j:integer=0;
   Aux:Tline;
   Track:TDataPath;
   WantedPose :TPose;
   PosAux:TPosi;
begin
    Vnomin:=VEL_FAST;
    Vfinal:=VEL_DEC;

    Startx.x :=round((X_loc*SCALE+Wid_map/2-Sizec/2)/Sizec);
    Startx.y :=round((Y_loc*SCALE+Heig_map/2-Sizec/2)/Sizec);
    Goalx.x :=round((Path.Xi+Wid_map/2-Sizec/2)/Sizec);
    Goalx.y :=round((Path.Yi+Heig_map/2-Sizec/2)/Sizec);
    //form1.MemoAstar.lines.add(floattostr(Startx.x)+' '+floattostr(Startx.y));

    if (isPassable(Startx, Startx.X,Startx.Y)) and (isPassable(Goalx, Goalx.X,Goalx.Y)) then begin
        Heap := Astar(Map, Startx, Goalx,Sizec,OBSTACLE);
    end else begin
        Form1.MemoAstar.Lines.add('Bloqueado o caminho');
    end;

    j := Length(Heap)-1;
    if j > 0 then begin
    if (j-3 > 0) then begin

         Aux.Xi := (Heap[j].X*Sizec+Sizec/2-Wid_map/2)/SCALE;
         Aux.Xf := (Heap[j-3].X*Sizec+Sizec/2-Wid_map/2)/SCALE;
         Aux.Yi := (Heap[j].Y*Sizec+Sizec/2-Heig_map/2)/SCALE;
         Aux.Yf := (Heap[j-3].Y*Sizec+Sizec/2-Heig_map/2)/SCALE;

         Track := ConfigTrack(Aux);

         WantedPose.X := (Heap[j-3].X*Sizec+Sizec/2-Wid_map/2)/SCALE;
         WantedPose.Y := (Heap[j-3].Y*Sizec+Sizec/2-Heig_map/2)/SCALE;
         WantedPose.Th := Th;
         form1.MemoAstar.lines.add(floattostr(FMover_xy));
         Speed := GoToXYTheta(Track, WantedPose,0);

         //Speed := FollowLine(Track.Xi, Track.X_act, Track.Y_act, Track.delta);
    end else begin
        // Vnomin:=VEL_ENTER;
        // Vfinal:=0;
        // Th_final:=Th;
       //  Th_nomin:=Th;
         Aux.Xi := (Heap[j].X*Sizec+Sizec/2-Wid_map/2)/SCALE;
         Aux.Xf := (Path.Xi)/SCALE;
         Aux.Yi := (Heap[j].Y*Sizec+Sizec/2-Heig_map/2)/SCALE;
         Aux.Yf := (Path.Yi)/SCALE;

         Track := ConfigTrack(Aux);

         WantedPose.X := (Path.Xi)/SCALE;
         WantedPose.Y := (Path.Yi)/SCALE;
         WantedPose.Th := Th;

         Speed := GoToXYTheta(Track, WantedPose,0);

         //Speed := FollowLine(Track.Xi, Track.X_act, Track.Y_act, Track.delta);

    end;
    end;


end;

procedure MachAtoB(M1, M2: TMachine);
var
   Path, Aux: TLine;
   Track : TDataPath;
begin
    if (StateMtoM = 0) then begin
    if (((X_Loc*SCALE > M1.Points.Xi + 4) or (X_Loc*SCALE < M1.Points.Xi - 4)) or
        ((Y_Loc*SCALE > M1.Points.Yi + 4) or (Y_Loc*SCALE < M1.Points.Yi - 4))) then begin
        Path.Xi:=M1.Points.Xf;
        Path.Yi:=M1.Points.Yf;
        Path.Xf:=M1.Points.Xi;
        Path.Yf:=M1.Points.Yi;
        Th_nomin:=M1.Th;
        Th_final:=M1.Th;
        EnterMach(Path,M1.Th);
    end else begin
        StateMtoM:=1;
        EnterMach_Flag:=False;
    end;
    end;

    if (StateMtoM = 1) then begin
    if ((X_Loc*SCALE > M2.Points.Xi + 4) or (X_Loc*SCALE < M2.Points.Xi - 4) or
        (Y_Loc*SCALE > M2.Points.Yi + 4) or (Y_Loc*SCALE < M2.Points.Yi - 4)) then begin
        servo_left:=2;
        servo_right:=2;
        Path.Xi:=M2.Points.Xi;
        Path.Yi:=M2.Points.Yi;
        Path.Xf:=M1.Points.Xi;
        Path.Yf:=M1.Points.Yi;
        Th_nomin:=M2.Th;
        Th_final:=M2.Th;

        if not(IsValidPos(Map,Robo_r,Path.Xi, Path.Yi)) then begin

           // ProjP := True;

            //if not(ProjP) then begin
                PosAux :=ProjectPoint(Path.Xi,Path.Yi);
           // end;
   //         Bmp_map.SetPixel(round((PosAux.X+Wid_map/2-Sizec/2)/Sizec),round((PosAux.Y+Heig_map/2-Sizec/2)/Sizec),BGRA(0,0,0));
//  PaintBitmap(Bmp_Map,5,320);
            Path.Xi:=PosAux.X/SCALE;
            Path.Yi:=PosAux.Y/SCALE;
            Path.Xf:=PosAux.X/SCALE;
            Path.Yf:=PosAux.Y/SCALE;

            Track:=ConfigTrack(Path);

            if (abs(Track.X_act)<0.005) then begin
                Speed:=PosControl(Track.X_act,Track.Y_act,M2.Th,Track.delta);
            end else begin
                Path.Xi:=PosAux.X;
                Path.Yi:=PosAux.Y;
                Path.Xf:=PosAux.X;
                Path.Yf:=PosAux.Y;

                AstarPath(Path,M2.Th);
            end;
        end else begin
            AstarPath(Path,M2.Th);
            ProjP := False;
        end;
    end else begin
        StateMtoM:=2; FMover_xy:=1;
    end;
    end;

    if (StateMtoM = 2) then begin
    if ((X_Loc*SCALE > M2.Points.Xi + 3) or (X_Loc*SCALE < M2.Points.Xi - 3) or
        (Y_Loc*SCALE > M2.Points.Yi + 3) or (Y_Loc*SCALE < M2.Points.Yi - 3) or
         (abs(NormalAng(DiffAngle(M2.Th,Th_loc))) > 5*pi/180)) then begin
        servo_left:=1;
        servo_right:=1;
        Path.Xi:=M2.Points.Xf/SCALE;
        Path.Yi:=M2.Points.Yf/SCALE;
        Path.Xf:=M2.Points.Xi/SCALE;
        Path.Yf:=M2.Points.Yi/SCALE;
        Track:=ConfigTrack(Path);
        Speed:=PosControl(Track.X_act,Track.Y_act,M2.Th,Track.delta);
    end else begin
        StateMtoM:=3;
    end;
    end;

    if (StateMtoM = 3) then begin
    if ((X_Loc*SCALE > M2.Points.Xf + 2) or (X_Loc*SCALE < M2.Points.Xf - 2) or
        (Y_Loc*SCALE > M2.Points.Yf + 2) or (Y_Loc*SCALE < M2.Points.Yf - 2)) then begin
        Path.Xi:=M2.Points.Xi;
        Path.Yi:=M2.Points.Yi;
        Path.Xf:=M2.Points.Xf;
        Path.Yf:=M2.Points.Yf;
        Th_nomin:=M2.Th;
        Th_final:=M2.Th;
        EnterMach(Path,M2.Th);
     end else begin
         StateMtoM := 4;
         EnterMach_Flag:=False;
     end;
    end;

    if StateMtoM = 4 then begin//((X_Loc*SCALE < M2.Points.Xf + 1) and (X_Loc*SCALE > M2.Points.Xf - 1) and
        //(Y_Loc*SCALE < M2.Points.Yf + 1) and (Y_Loc*SCALE > M2.Points.Yf - 1)) then begin
        Path.Xi:=M2.Points.Xi/SCALE;
        Path.Yi:=M2.Points.Yi/SCALE;
        Path.Xf:=M2.Points.Xf/SCALE;
        Path.Yf:=M2.Points.Yf/SCALE;

        Track := ConfigTrack(Path);

        Speed := PosControl(Track.X_act,Track.Y_act,M2.Th,Track.delta);

        Garras_use:=True;
    end;


end;

procedure TForm1.AssignValClick(Sender: TObject);
var Start2, end2: qword;
begin
  N_Livre:=0;
  Wid_map:= strtofloat(Width_map.Text);
  Heig_map:= strtofloat(Height_map.Text);
  Sizec := StrtoFloat(Size_grid.Text);
  Dist_Cen := StrtoFloat(DisCen.Text);
  Wid_obs := strtofloat(Width_obs.Text);
  Heig_obs:= strtofloat(Height_obs.Text);

  Start2:=GetTickCount64;

  Bmp_Map:= TBGRABitmap.Create(round(Wid_map/Sizec), round(Heig_map/Sizec), BGRA(0,255,0));
  //Bmp_map.FillEllipseAntialias(15,15,1,1,BGRA(255,0,0));
  MakeMap();

  MinkowskySum(Map,StrToFloat(Robot.Text));

  end2:=getTickCount64-start2;
  //ShowMessage(inttostr(end2));
  Memo1.Lines.add('T: ' + inttostr(end2)+ 'N: ' +inttostr(N_livre));
end;

procedure TForm1.Button1Click(Sender: TObject);
var
   DistR:Double;
   TestePos : Tposi;
   Start2, end2: qword;
begin
   N_Livre:=0;
  //testex:=testex+1;//round(300/5);
  //testey:=testey+1;//round(200/5);
   R2x := StrToFloat(Robot2X.Text);
   R2y := StrToFloat(Robot2Y.Text);
   R2_r := 25;

   DistR := sqrt((X_loc-R2X)*(X_loc-R2X)+(Y_loc-R2Y)*(Y_loc-R2Y));

  Start2:=GetTickCount64;
  ChangeMap();
  MinkowskySum(Map,StrToFloat(Robot.Text));

  end2:=getTickCount64-start2;
  //ShowMessage(inttostr(end2));
  Memo1.Lines.add('T: ' + inttostr(end2)+ 'N: ' +inttostr(N_livre));
  //TestePos := ProjectPoint(113,-1);

//  IsValidPos(Map,Robo_r,);


  MatrixToBitmap(Bmp_Map,map,OBSTACLE);
  //Bmp_map.SetPixel(round((TestePos.X+Wid_map/2-Sizec/2)/Sizec),round((TestePos.Y+Heig_map/2-Sizec/2)/Sizec),BGRA(0,0,0));
  PaintBitmap(Bmp_Map,5,320);

  MemoAstar.Lines.add(Floattostr(TestePos.X+150)+' '+floattoSTR(TestePos.Y+100));
end;

procedure TForm1.DefMachClick(Sender: TObject);
begin
  //Armazém entrada
   Machine[0].Points.Xi:=StrToFloat(Form1.X_A_M1.Text);
   Machine[0].Points.Yi:=StrToFloat(Form1.Y_A_M1.Text);
   Machine[0].Points.Xf:=StrToFloat(Form1.X_M1.Text);
   Machine[0].Points.Yf:=StrToFloat(Form1.Y_M1.Text);
   Machine[0].Th:=pi/2;

   Machine[1].Points.Xi:=StrToFloat(Form1.X_A_M2.Text);
   Machine[1].Points.Yi:=StrToFloat(Form1.Y_A_M2.Text);
   Machine[1].Points.Xf:=StrToFloat(Form1.X_M2.Text);
   Machine[1].Points.Yf:=StrToFloat(Form1.Y_M2.Text);
   Machine[1].Th:=pi/2;
  //
   Machine[2].Points.Xi:=StrToFloat(Form1.X_A_M3.Text);
   Machine[2].Points.Yi:=StrToFloat(Form1.Y_A_M3.Text);
   Machine[2].Points.Xf:=StrToFloat(Form1.X_M3.Text);
   Machine[2].Points.Yf:=StrToFloat(Form1.Y_M3.Text);
   Machine[2].Th:=pi/2;
  //
   Machine[3].Points.Xi:=StrToFloat(Form1.X_A_M4.Text);
   Machine[3].Points.Yi:=StrToFloat(Form1.Y_A_M4.Text);
   Machine[3].Points.Xf:=StrToFloat(Form1.X_M4.Text);
   Machine[3].Points.Yf:=StrToFloat(Form1.Y_M4.Text);
   Machine[3].Th:=pi/2;
  //
   Machine[4].Points.Xi:=StrToFloat(Form1.X_A_M5.Text);
   Machine[4].Points.Yi:=StrToFloat(Form1.Y_A_M5.Text);
   Machine[4].Points.Xf:=StrToFloat(Form1.X_M5.Text);
   Machine[4].Points.Yf:=StrToFloat(Form1.Y_M5.Text);
   Machine[4].Th:=pi/2;
  //
   //Maquina Esquerda
   Machine[5].Points.Xi:=StrToFloat(Form1.X_A_M6.Text);
   Machine[5].Points.Yi:=StrToFloat(Form1.Y_A_M6.Text);
   Machine[5].Points.Xf:=StrToFloat(Form1.X_M6.Text);
   Machine[5].Points.Yf:=StrToFloat(Form1.Y_M6.Text);
   Machine[5].Th:=0;
  //
   Machine[6].Points.Xi:=StrToFloat(Form1.X_A_M7.Text);
   Machine[6].Points.Yi:=StrToFloat(Form1.Y_A_M7.Text);
   Machine[6].Points.Xf:=StrToFloat(Form1.X_M7.Text);
   Machine[6].Points.Yf:=StrToFloat(Form1.Y_M7.Text);
   Machine[6].Th:=0;
  //
   Machine[7].Points.Xi:=StrToFloat(Form1.X_A_M8.Text);
   Machine[7].Points.Yi:=StrToFloat(Form1.Y_A_M8.Text);
   Machine[7].Points.Xf:=StrToFloat(Form1.X_M8.Text);
   Machine[7].Points.Yf:=StrToFloat(Form1.Y_M8.Text);
   Machine[7].Th:=pi;
  //
   Machine[8].Points.Xi:=StrToFloat(Form1.X_A_M9.Text);
   Machine[8].Points.Yi:=StrToFloat(Form1.Y_A_M9.Text);
   Machine[8].Points.Xf:=StrToFloat(Form1.X_M9.Text);
   Machine[8].Points.Yf:=StrToFloat(Form1.Y_M9.Text);
   Machine[8].Th:=pi;
  //
   //Maquina Direita
   Machine[9].Points.Xi:=StrToFloat(Form1.X_A_M10.Text);
   Machine[9].Points.Yi:=StrToFloat(Form1.Y_A_M10.Text);
   Machine[9].Points.Xf:=StrToFloat(Form1.X_M10.Text);
   Machine[9].Points.Yf:=StrToFloat(Form1.Y_M10.Text);
   Machine[9].Th:=0;
  //
   Machine[10].Points.Xi:=StrToFloat(Form1.X_A_M11.Text);
   Machine[10].Points.Yi:=StrToFloat(Form1.Y_A_M11.Text);
   Machine[10].Points.Xf:=StrToFloat(Form1.X_M11.Text);
   Machine[10].Points.Yf:=StrToFloat(Form1.Y_M11.Text);
   Machine[10].Th:=0;
  //
   Machine[11].Points.Xi:=StrToFloat(Form1.X_A_M12.Text);
   Machine[11].Points.Yi:=StrToFloat(Form1.Y_A_M12.Text);
   Machine[11].Points.Xf:=StrToFloat(Form1.X_M12.Text);
   Machine[11].Points.Yf:=StrToFloat(Form1.Y_M12.Text);
   Machine[11].Th:=pi;
  //
   Machine[12].Points.Xi:=StrToFloat(Form1.X_A_M13.Text);
   Machine[12].Points.Yi:=StrToFloat(Form1.Y_A_M13.Text);
   Machine[12].Points.Xf:=StrToFloat(Form1.X_M13.Text);
   Machine[12].Points.Yf:=StrToFloat(Form1.Y_M13.Text);
   Machine[12].Th:=pi;
  //
   //Armazem Saida
   Machine[13].Points.Xi:=StrToFloat(Form1.X_A_M14.Text);
   Machine[13].Points.Yi:=StrToFloat(Form1.Y_A_M14.Text);
   Machine[13].Points.Xf:=StrToFloat(Form1.X_M14.Text);
   Machine[13].Points.Yf:=StrToFloat(Form1.Y_M14.Text);
   Machine[13].Th:=-pi/2;
  //
   Machine[14].Points.Xi:=StrToFloat(Form1.X_A_M15.Text);
   Machine[14].Points.Yi:=StrToFloat(Form1.Y_A_M15.Text);
   Machine[14].Points.Xf:=StrToFloat(Form1.X_M15.Text);
   Machine[14].Points.Yf:=StrToFloat(Form1.Y_M15.Text);
   Machine[14].Th:=-pi/2;
  //
   Machine[15].Points.Xi:=StrToFloat(Form1.X_A_M16.Text);
   Machine[15].Points.Yi:=StrToFloat(Form1.Y_A_M16.Text);
   Machine[15].Points.Xf:=StrToFloat(Form1.X_M16.Text);
   Machine[15].Points.Yf:=StrToFloat(Form1.Y_M16.Text);
   Machine[15].Th:=-pi/2;
  //
   Machine[16].Points.Xi:=StrToFloat(Form1.X_A_M17.Text);
   Machine[16].Points.Yi:=StrToFloat(Form1.Y_A_M17.Text);
   Machine[16].Points.Xf:=StrToFloat(Form1.X_M17.Text);
   Machine[16].Points.Yf:=StrToFloat(Form1.Y_M17.Text);
   Machine[16].Th:=-pi/2;
  //
   Machine[17].Points.Xi:=StrToFloat(Form1.X_A_M18.Text);
   Machine[17].Points.Yi:=StrToFloat(Form1.Y_A_M18.Text);
   Machine[17].Points.Xf:=StrToFloat(Form1.X_M18.Text);
   Machine[17].Points.Yf:=StrToFloat(Form1.Y_M18.Text);
   Machine[17].Th:=-pi/2;
end;

procedure TForm1.CalculateClick(Sender: TObject);
var
     i, start2, end2:integer;
     Heap : TNodeArray;
     Startx, Goalx : Tnode;
     Path: TLine;
     desl:double=0;
begin
            Start2:=GetTickCount;

            Startx.x :=round((StrToFloat(x_ini.Text)+Wid_map/2-Sizec/2)/Sizec);
            Startx.y :=round((StrToFloat(y_ini.Text)+Heig_map/2-Sizec/2)/Sizec);
            Goalx.x :=round((StrToFloat(x_goal.Text)+Wid_map/2-Sizec/2)/Sizec);
            Goalx.y :=round((StrToFloat(y_goal.Text)+Heig_map/2-Sizec/2)/Sizec);


            if (isPassable(Startx, Startx.X,Startx.Y)) and (isPassable(Goalx, Goalx.X,Goalx.Y)) then begin
                    heap := Astar(Map, Startx, Goalx,Sizec,OBSTACLE);
            end else begin
                    MemoAstar.Lines.add('Bloqueado o caminho');
            end;

            end2:=getTickCount-start2;
            Memo1.lines.add('Time ms: '+inttostr(end2));

            for i := 0 to Length(Heap)-1 do begin
                //Bmp_Map.DrawLine(round((-113+Wid_map/2-Sizec/2)/Sizec),round((53+Heig_map/2-Sizec/2)/Sizec),round((-113+Wid_map/2-Sizec/2)/Sizec),round((78+Heig_map/2-Sizec/2)/Sizec),BGRA(0,0,255),true);
                //Bmp_Map.DrawLine(round((113+Wid_map/2-Sizec/2)/Sizec),round((-53+Heig_map/2-Sizec/2)/Sizec),round((113+Wid_map/2-Sizec/2)/Sizec),round((-78+Heig_map/2-Sizec/2)/Sizec),BGRA(0,0,255),true);
                Bmp_Map.SetPixel(Heap[i].x,Heap[i].Y, BGRA(0,0,0));
               MemoAstar.Lines.add(floattostr(sizec/2+sizec*(Heap[i].x)-150) +' '+floattostr(sizec/2+sizec*(Heap[i].Y)-100));
                if Length(Heap)-1>=0 then desl := desl + Heap[i].Cost;
            end;

            memo1.lines.Add(floattostr(desl));

            PaintBitmap(Bmp_Map, 5, 320);
           // Path.Xi :=23.25; Path.Yi:=176.25; path.Xf := 299; Path.yf:=199;

           // MemoAstar.Lines.add(floattostr(DecompPath(Dist,Path,22,2.5)[0].xf)+floattostr(DecompPath(Dist,Path,22,2.5)[0].Yf));
           //  MemoAstar.Lines.add(floattostr(DecompPath(Dist,Path,22,2.5)[1].xi)+floattostr(DecompPath(Dist,Path,22,2.5)[1].Yi));
           // MemoAstar.Lines.add(floattostr(DecompPath(Dist,Path,22,2.5)[1].xf)+floattostr(DecompPath(Dist,Path,22,2.5)[1].Yf));
           // MemoAstar.Lines.add(floattostr(DecompPath(Dist,Path,22,2.5)[2].xi)+floattostr(DecompPath(Dist,Path,22,2.5)[2].Yi));

end;

procedure TForm1.ConnectClick(Sender: TObject);
begin
  StateMtoM:=0;
  EnterMach_Flag:=false;
  Set_start:=false;
  Manga1_State := 0;
  servo_left:=0;
  servo_right:=0;
  FMover_xy:=1;

  if (not Form1.RealRobot.Checked ) then begin
      SimTwoUDP.Connect(IP_R1.Text,strtoint(Port_R1.Text));
      SimTwoUDP.Listen(strtoint(Port_R1.Text));
      SimTwoUDPR2.Connect(IP_R2.Text,strtoint(Port_R2.Text));
      SimTwoUDPR2.Listen(strtoint(Port_R2.Text));
  end else begin
      UDPRob.Connect(IP_R1.text,9003);
      UDPRob.Listen(9003);
      UDPTask.Connect(IP_R1.text,9022);
      UDPTask.Listen(9022);
  end;

end;

procedure TForm1.DisconnectClick(Sender: TObject);
begin

  ActionOrder:=3;
  SimTwoUDP.Disconnect();
  SimTwoUDPR2.Disconnect();
  UDPRob.Disconnect();
  UDPTask.Disconnect();
end;

procedure TForm1.Draw_mapClick(Sender: TObject);
var
     X,Y : integer;
begin

     MatrixToBitmap(Bmp_Map,Map, OBSTACLE);
     PaintBitmap(Bmp_Map, 5, 320);
     Bmp_Map.free;
     Grid_repSeries.clear;
     Grid_repSeries1.clear;
     Grid_repSeries2.clear;

     X:=0;
     while(X < Map.NumCols) do
     begin
          Y:=0;
          while (Y < Map.NumRows) do
          begin
               if (Map.getv(Y,X) = 1.5) or (Map.getv(Y,X) = 3) or(Map.getv(Y,X) = 2) then
               begin
               Grid_repSeries.AddXY(StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(X),StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(Y));
               end
               else if Map.getv(Y,X) = OBSTACLE then
               begin
               Grid_repSeries1.AddXY(StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(X),StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(Y));
               end
               else
               begin
               Grid_repSeries2.AddXY(StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(X),StrtoFloat(Size_grid.Text)/2+StrtoFloat(Size_grid.Text)*(Y));
               end;
               Y := Y + 1;
          end;
          X := X + 1;
     end;
end;

procedure TForm1.FormCreate(Sender: TObject);
begin

   //Armazém entrada
   Machine[0].Points.Xi:=StrToFloat(Form1.X_A_M1.Text);
   Machine[0].Points.Yi:=StrToFloat(Form1.Y_A_M1.Text);
   Machine[0].Points.Xf:=StrToFloat(Form1.X_M1.Text);
   Machine[0].Points.Yf:=StrToFloat(Form1.Y_M1.Text);
   Machine[0].Th:=pi/2;

   Machine[1].Points.Xi:=StrToFloat(Form1.X_A_M2.Text);
   Machine[1].Points.Yi:=StrToFloat(Form1.Y_A_M2.Text);
   Machine[1].Points.Xf:=StrToFloat(Form1.X_M2.Text);
   Machine[1].Points.Yf:=StrToFloat(Form1.Y_M2.Text);
   Machine[1].Th:=pi/2;
  //
   Machine[2].Points.Xi:=StrToFloat(Form1.X_A_M3.Text);
   Machine[2].Points.Yi:=StrToFloat(Form1.Y_A_M3.Text);
   Machine[2].Points.Xf:=StrToFloat(Form1.X_M3.Text);
   Machine[2].Points.Yf:=StrToFloat(Form1.Y_M3.Text);
   Machine[2].Th:=pi/2;
  //
   Machine[3].Points.Xi:=StrToFloat(Form1.X_A_M4.Text);
   Machine[3].Points.Yi:=StrToFloat(Form1.Y_A_M4.Text);
   Machine[3].Points.Xf:=StrToFloat(Form1.X_M4.Text);
   Machine[3].Points.Yf:=StrToFloat(Form1.Y_M4.Text);
   Machine[3].Th:=pi/2;
  //
   Machine[4].Points.Xi:=StrToFloat(Form1.X_A_M5.Text);
   Machine[4].Points.Yi:=StrToFloat(Form1.Y_A_M5.Text);
   Machine[4].Points.Xf:=StrToFloat(Form1.X_M5.Text);
   Machine[4].Points.Yf:=StrToFloat(Form1.Y_M5.Text);
   Machine[4].Th:=pi/2;
  //
   //Maquina Esquerda
   Machine[5].Points.Xi:=StrToFloat(Form1.X_A_M6.Text);
   Machine[5].Points.Yi:=StrToFloat(Form1.Y_A_M6.Text);
   Machine[5].Points.Xf:=StrToFloat(Form1.X_M6.Text);
   Machine[5].Points.Yf:=StrToFloat(Form1.Y_M6.Text);
   Machine[5].Th:=0;
  //
   Machine[6].Points.Xi:=StrToFloat(Form1.X_A_M7.Text);
   Machine[6].Points.Yi:=StrToFloat(Form1.Y_A_M7.Text);
   Machine[6].Points.Xf:=StrToFloat(Form1.X_M7.Text);
   Machine[6].Points.Yf:=StrToFloat(Form1.Y_M7.Text);
   Machine[6].Th:=0;
  //
   Machine[7].Points.Xi:=StrToFloat(Form1.X_A_M8.Text);
   Machine[7].Points.Yi:=StrToFloat(Form1.Y_A_M8.Text);
   Machine[7].Points.Xf:=StrToFloat(Form1.X_M8.Text);
   Machine[7].Points.Yf:=StrToFloat(Form1.Y_M8.Text);
   Machine[7].Th:=pi;
  //
   Machine[8].Points.Xi:=StrToFloat(Form1.X_A_M9.Text);
   Machine[8].Points.Yi:=StrToFloat(Form1.Y_A_M9.Text);
   Machine[8].Points.Xf:=StrToFloat(Form1.X_M9.Text);
   Machine[8].Points.Yf:=StrToFloat(Form1.Y_M9.Text);
   Machine[8].Th:=pi;
  //
   //Maquina Direita
   Machine[9].Points.Xi:=StrToFloat(Form1.X_A_M10.Text);
   Machine[9].Points.Yi:=StrToFloat(Form1.Y_A_M10.Text);
   Machine[9].Points.Xf:=StrToFloat(Form1.X_M10.Text);
   Machine[9].Points.Yf:=StrToFloat(Form1.Y_M10.Text);
   Machine[9].Th:=0;
  //
   Machine[10].Points.Xi:=StrToFloat(Form1.X_A_M11.Text);
   Machine[10].Points.Yi:=StrToFloat(Form1.Y_A_M11.Text);
   Machine[10].Points.Xf:=StrToFloat(Form1.X_M11.Text);
   Machine[10].Points.Yf:=StrToFloat(Form1.Y_M11.Text);
   Machine[10].Th:=0;
  //
   Machine[11].Points.Xi:=StrToFloat(Form1.X_A_M12.Text);
   Machine[11].Points.Yi:=StrToFloat(Form1.Y_A_M12.Text);
   Machine[11].Points.Xf:=StrToFloat(Form1.X_M12.Text);
   Machine[11].Points.Yf:=StrToFloat(Form1.Y_M12.Text);
   Machine[11].Th:=pi;
  //
   Machine[12].Points.Xi:=StrToFloat(Form1.X_A_M13.Text);
   Machine[12].Points.Yi:=StrToFloat(Form1.Y_A_M13.Text);
   Machine[12].Points.Xf:=StrToFloat(Form1.X_M13.Text);
   Machine[12].Points.Yf:=StrToFloat(Form1.Y_M13.Text);
   Machine[12].Th:=pi;
  //
   //Armazem Saida
   Machine[13].Points.Xi:=StrToFloat(Form1.X_A_M14.Text);
   Machine[13].Points.Yi:=StrToFloat(Form1.Y_A_M14.Text);
   Machine[13].Points.Xf:=StrToFloat(Form1.X_M14.Text);
   Machine[13].Points.Yf:=StrToFloat(Form1.Y_M14.Text);
   Machine[13].Th:=-pi/2;
  //
   Machine[14].Points.Xi:=StrToFloat(Form1.X_A_M15.Text);
   Machine[14].Points.Yi:=StrToFloat(Form1.Y_A_M15.Text);
   Machine[14].Points.Xf:=StrToFloat(Form1.X_M15.Text);
   Machine[14].Points.Yf:=StrToFloat(Form1.Y_M15.Text);
   Machine[14].Th:=-pi/2;
  //
   Machine[15].Points.Xi:=StrToFloat(Form1.X_A_M16.Text);
   Machine[15].Points.Yi:=StrToFloat(Form1.Y_A_M16.Text);
   Machine[15].Points.Xf:=StrToFloat(Form1.X_M16.Text);
   Machine[15].Points.Yf:=StrToFloat(Form1.Y_M16.Text);
   Machine[15].Th:=-pi/2;
  //
   Machine[16].Points.Xi:=StrToFloat(Form1.X_A_M17.Text);
   Machine[16].Points.Yi:=StrToFloat(Form1.Y_A_M17.Text);
   Machine[16].Points.Xf:=StrToFloat(Form1.X_M17.Text);
   Machine[16].Points.Yf:=StrToFloat(Form1.Y_M17.Text);
   Machine[16].Th:=-pi/2;
  //
   Machine[17].Points.Xi:=StrToFloat(Form1.X_A_M18.Text);
   Machine[17].Points.Yi:=StrToFloat(Form1.Y_A_M18.Text);
   Machine[17].Points.Xf:=StrToFloat(Form1.X_M18.Text);
   Machine[17].Points.Yf:=StrToFloat(Form1.Y_M18.Text);
   Machine[17].Th:=-pi/2;

  // //Armazém entrada
  // Machine[0].Points.Xi:=-113;
  // Machine[0].Points.Yi:=78-StrToFloat(Form1.Robot.Text);
  // Machine[0].Points.Xf:=-113;
  // Machine[0].Points.Yf:=78;
  // Machine[0].Th:=pi/2;
  //
  // Machine[1].Points.Xi:=-95;
  // Machine[1].Points.Yi:=78-StrToFloat(Form1.Robot.Text);
  // Machine[1].Points.Xf:=-95;
  // Machine[1].Points.Yf:=78;
  // Machine[1].Th:=pi/2;
  ////
  // Machine[2].Points.Xi:=-77;
  // Machine[2].Points.Yi:=78-StrToFloat(Form1.Robot.Text);
  // Machine[2].Points.Xf:=-77;
  // Machine[2].Points.Yf:=78;
  // Machine[2].Th:=pi/2;
  ////
  // Machine[3].Points.Xi:=-59;
  // Machine[3].Points.Yi:=78-StrToFloat(Form1.Robot.Text);
  // Machine[3].Points.Xf:=-59;
  // Machine[3].Points.Yf:=78;
  // Machine[3].Th:=pi/2;
  ////
  // Machine[4].Points.Xi:=-41;
  // Machine[4].Points.Yi:=78-StrToFloat(Form1.Robot.Text);
  // Machine[4].Points.Xf:=-41;
  // Machine[4].Points.Yf:=78;
  // Machine[4].Th:=pi/2;
  ////
  // //Maquina Esquerda
  // Machine[5].Points.Xi:=-91-StrToFloat(Form1.Robot.Text);
  // Machine[5].Points.Yi:=10;
  // Machine[5].Points.Xf:=-91;
  // Machine[5].Points.Yf:=10;
  // Machine[5].Th:=0;
  ////
  // Machine[6].Points.Xi:=-91-StrToFloat(Form1.Robot.Text);
  // Machine[6].Points.Yi:=-10;
  // Machine[6].Points.Xf:=-91;
  // Machine[6].Points.Yf:=-10;
  // Machine[6].Th:=0;
  ////
  // Machine[7].Points.Xi:=-34+StrToFloat(Form1.Robot.Text);
  // Machine[7].Points.Yi:=10;
  // Machine[7].Points.Xf:=-34;
  // Machine[7].Points.Yf:=10;
  // Machine[7].Th:=pi;
  ////
  // Machine[8].Points.Xi:=-34+StrToFloat(Form1.Robot.Text);
  // Machine[8].Points.Yi:=-10;
  // Machine[8].Points.Xf:=-34;
  // Machine[8].Points.Yf:=-10;
  // Machine[8].Th:=pi;
  ////
  // //Maquina Direita
  // Machine[9].Points.Xi:=34-StrToFloat(Form1.Robot.Text);
  // Machine[9].Points.Yi:=10;
  // Machine[9].Points.Xf:=34;
  // Machine[9].Points.Yf:=10;
  // Machine[9].Th:=0;
  ////
  // Machine[10].Points.Xi:=34-StrToFloat(Form1.Robot.Text);
  // Machine[10].Points.Yi:=-10;
  // Machine[10].Points.Xf:=34;
  // Machine[10].Points.Yf:=-10;
  // Machine[10].Th:=0;
  ////
  // Machine[11].Points.Xi:=91+StrToFloat(Form1.Robot.Text);
  // Machine[11].Points.Yi:=10;
  // Machine[11].Points.Xf:=91;
  // Machine[11].Points.Yf:=10;
  // Machine[11].Th:=pi;
  ////
  // Machine[12].Points.Xi:=91+StrToFloat(Form1.Robot.Text);
  // Machine[12].Points.Yi:=-10;
  // Machine[12].Points.Xf:=91;
  // Machine[12].Points.Yf:=-10;
  // Machine[12].Th:=pi;
  ////
  // //Armazem Saida
  // Machine[13].Points.Xi:=41;
  // Machine[13].Points.Yi:=-78+StrToFloat(Form1.Robot.Text);
  // Machine[13].Points.Xf:=41;
  // Machine[13].Points.Yf:=-78;
  // Machine[13].Th:=-pi/2;
  ////
  // Machine[14].Points.Xi:=59;
  // Machine[14].Points.Yi:=-78+StrToFloat(Form1.Robot.Text);
  // Machine[14].Points.Xf:=59;
  // Machine[14].Points.Yf:=-78;
  // Machine[14].Th:=-pi/2;
  ////
  // Machine[15].Points.Xi:=77;
  // Machine[15].Points.Yi:=-78+StrToFloat(Form1.Robot.Text);
  // Machine[15].Points.Xf:=77;
  // Machine[15].Points.Yf:=-78;
  // Machine[15].Th:=-pi/2;
  ////
  // Machine[16].Points.Xi:=95;
  // Machine[16].Points.Yi:=-78+StrToFloat(Form1.Robot.Text);
  // Machine[16].Points.Xf:=95;
  // Machine[16].Points.Yf:=-78;
  // Machine[16].Th:=-pi/2;
  ////
  // Machine[17].Points.Xi:=113;
  // Machine[17].Points.Yi:=-78+StrToFloat(Form1.Robot.Text);
  // Machine[17].Points.Xf:=113;
  // Machine[17].Points.Yf:=-78;
  // Machine[17].Th:=-pi/2;
  ActionOrder:=3;
  DefineMap();
end;

procedure TForm1.FormShow(Sender: TObject);
begin
  //SimTwoUDP.Listen(9810);
end;

procedure TForm1.SimTwoUDPError(const msg: string; aSocket: TLSocket);
begin

end;

procedure TForm1.SimTwoUDPReceive(aSocket: TLSocket);
var
     IniValid, FinValid, Decomp : boolean;
     MultPath : TLineArray;
     Track : TDataPath;
     Message : string;
     i : integer = 0;
     j : integer;
     Heap : TNodeArray;
     Startx, Goalx : Tnode;
     Aux : TLine;
begin
    // if not(Decomp) then begin
    //     IniValid:=IsPosValid(Dist, StrToFloat(Robot.Text), Path.Xi+150, Path.Yi+100);
    //     FinValid:=IsPosValid(Dist, StrToFloat(Robot.Text), Path.Xf+150, Path.Yf+100);

    //     MultPath := DecompPath(Dist,Path,StrToFloat(Robot.Text),Sizec);

    //     Decomp := True;
    // end;

     SimTwoUDP.GetMessage(Message);
     Memo1.Lines.Add(Message);
     X_loc := StrToFloat(ExtractDelimited(2,Message,[':']));
     Y_loc := StrToFloat(ExtractDelimited(4,Message,[':']));
     Th_loc := NormalAng(StrToFloat(ExtractDelimited(6,Message,[':'])));

    if Set_start=false then begin
         Pos_ini.Points.Xi:=X_loc*SCALE;
         Pos_ini.Points.Yi:=Y_loc*SCALE;
         Pos_ini.Points.Xf:=X_loc*SCALE;
         Pos_ini.Points.Yf:=Y_loc*SCALE;
         pos_ini.Th:=Th_loc;
         Set_start:=True;
     end;

     if (Manga1_State = 0) then begin
         MachAtoB(Pos_ini,Machine[0]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(1);
             //Garras_use := False;  EnterMach_Flag:=False;
             //Manga1_State :=1;
             //StateMtoM:=0;
         end;
     end;

     if (Manga1_State = 1) then begin
         MachAtoB(Machine[0],Machine[17]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(0);
             //Garras_use := False;  EnterMach_Flag:=False;
             //Manga1_State :=2;
             //StateMtoM:=0;
         end;
     end;

     if (Manga1_State = 2) then begin
         MachAtoB(Machine[17],Machine[1]);
         if(Garras_use=True) then begin
            //Control Garra
            ControlClaw(1);
             // Manga1_State :=3; Garras_use := False; EnterMach_Flag:=False;
            // StateMtoM:=0;
         end;
     end;

     if (Manga1_State = 3) then begin
         MachAtoB(Machine[1],Machine[16]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(0);
             //Manga1_State :=4; EnterMach_Flag:=False;
             //StateMtoM:=0; Garras_use := False;
         end;
     end;

     if (Manga1_State = 4) then begin
         MachAtoB(Machine[16],Machine[2]);
         if(Garras_use=True) then begin
            //Control Garra
            ControlClaw(1);
             //    Manga1_State :=5; Garras_use := False; EnterMach_Flag:=False;
          //   StateMtoM:=0;
         end;
     end;

     if (Manga1_State = 5) then begin
         MachAtoB(Machine[2],Machine[15]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(0);
             //Manga1_State :=6; Garras_use := False;
             //StateMtoM:=0; EnterMach_Flag:=False;
         end;
     end;

     if (Manga1_State = 6) then begin
         MachAtoB(Machine[15],Machine[3]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(1);
             //  Manga1_State :=7;
             //StateMtoM:=0; Garras_use := False; EnterMach_Flag:=False;
         end;
     end;

     if (Manga1_State = 7) then begin
         MachAtoB(Machine[3],Machine[14]);
         if(Garras_use=True) then begin
            //Control Garra
            ControlClaw(0);
             // Manga1_State :=8;
            // StateMtoM:=0; Garras_use := False; EnterMach_Flag:=False;
         end;
     end;

     if (Manga1_State = 8) then begin
         MachAtoB(Machine[14],Machine[4]);
         if(Garras_use=True) then begin
            //Control Garra
            ControlClaw(1);
             //   Manga1_State :=9; EnterMach_Flag:=False;
            // StateMtoM:=0; Garras_use := False;
         end;
     end;

     if (Manga1_State = 9) then begin
         MachAtoB(Machine[4],Machine[13]);
         if(Garras_use=True) then begin
            //Control Garra
             ControlClaw(0);
             //Manga1_State :=10; EnterMach_Flag:=False;
             //StateMtoM:=0; Garras_use := False;
         end;
     end;
    // Path.Xi:=-116.75;
     //Path.Xf:=118.75/100;
     //Path.Yi:=-66.75;
     //Path.Yf:=0;
     //EnterMach(Path,0);
       // AstarPath(Path,pi/2);
    // //if (i < 3) then begin
    // //    Track := ConfigTrack(MultPath[i]);
    //// end else begin
    //     // Next State
    //
    //// end;
    //
    // if (IniValid = False) and (i = 0) then begin
    //     Track := ConfigTrack(MultPath[0]);
    //     Speed := FollowLine(Track.Xi, Track.X_act, Track.Y_act, Track.delta);
    //     if Track.X_act > -0.005 then i:=i+1;
    // end else begin
    //     if i=0 then i:=i+1;
    // end;
    //
    // if (i = 1) then begin
    //     Startx.x :=round((X_loc+150-Sizec/2)/Sizec);
    //     Startx.y :=round((Y_loc+100-Sizec/2)/Sizec);
    //     Goalx.x :=round((MultPath[1].Xf+150-Sizec/2)/Sizec);
    //     Goalx.y :=round((MultPath[1].Yf+100-Sizec/2)/Sizec);
    //
    //     if (isPassable(Startx, Startx.X,Startx.Y)) and (isPassable(Goalx, Goalx.X,Goalx.Y)) then begin
    //         Heap := Astar(Map, Startx, Goalx, Sizec, OBSTACLE);
    //     end else begin
    //         MemoAstar.Lines.add('Bloqueado o caminho');
    //     end;
    //
    //     j := Length(Heap)-1;
    //
    //     if (j-3 > 0) then begin
    //     Aux.Xi := Heap[j].X*Sizec+Sizec/2-150;
    //     Aux.Xf := Heap[j-3].X*Sizec+Sizec/2-150;
    //     Aux.Yi := Heap[j].Y*Sizec+Sizec/2-100;
    //     Aux.Yf := Heap[j-3].Y*Sizec+Sizec/2-100;
    //
    //     Track := ConfigTrack(Aux);
    //
    //     Speed := FollowLine(Track.Xi, Track.X_act, Track.Y_act, Track.delta);
    //     end else begin
    //          i:=i+1;
    //     end;
    //
    // end;
      SimTwoUDP.SendMessage('R:'+#10+Robnum.Text+#10+'V:'+#10+FloatToStr(Speed.V)+#10+'Vn:'+#10+FloatToStr(Speed.Vn)+#10+'W:'+#10+FloatToStr(Speed.W)+#10,'127.0.0.1');


end;

procedure TForm1.SimTwoUDPR2Receive(aSocket: TLSocket);
var
    Message : string;
    DistR : Double;
begin
     SimTwoUDPR2.GetMessage(Message);

     R2X := StrToFloat(ExtractDelimited(2,Message,[':']));
     R2Y := StrToFloat(ExtractDelimited(4,Message,[':']));
     R2_r:= 25;

     DistR := sqrt((X_loc-R2X)*(X_loc-R2X)+(Y_loc-R2Y)*(Y_loc-R2Y));

     if(DistR < 1.5) then begin
         ChangeMap();
         MinkowskySum(Map,Robo_r);
         BuildM:=True;
     end else begin
         if BuildM then begin
             Bmp_Map:= TBGRABitmap.Create(round(Wid_map/Sizec), round(Heig_map/Sizec), BGRA(0,255,0));
             MakeMap();
             MinkowskySum(Map,Robo_r);
             BuildM:=False;
         end;
     end;

     // MatrixToBitmap(Bmp_Map,map,OBSTACLE);
 // Bmp_map.SetPixel(round((TestePos.X+Wid_map/2-Sizec/2)/Sizec),round((TestePos.Y+Heig_map/2-Sizec/2)/Sizec),BGRA(0,0,0));
  //PaintBitmap(Bmp_Map,5,320);


     //Th_loc := NormalAng(StrToFloat(ExtractDelimited(6,Message,[':'])));

end;

procedure TForm1.UDPRobReceive(aSocket: TLSocket);
begin

     RecievePosMessage(0);

     V_Debug.text := floattostr(Speed.V);
     Vn_Debug.text := floattostr(Speed.Vn);
     W_Debug.text := floattostr(Speed.W);

     X_Debug.text := floattostr(X_loc);
     Y_Debug.text := floattostr(Y_loc);
     TH_Debug.text := floattostr(TH_loc);

     LineConfig.Xi:=X_loc;
     LineConfig.Yi:=Y_loc;
     LineConfig.Xf:=FinalPose.X;
     LineConfig.Yf:=FinalPose.Y;

     TrackFollow := ConfigTrack(LineConfig);

     case ActionOrder of       //    por em m
         0: begin
             form1.Memo1.Append('State 0');
             Speed := GoToXYTheta(TrackFollow, FinalPose,0);

             //ControlClaw(ClawPos);
         end;
         1: begin
             Vnomin:=VEL_FAST;
             Vfinal:=VEL_DEC;
             form1.Memo1.Append('State 1');
             Speed := FollowLine(TrackFollow.Xi, TrackFollow.X_act, TrackFollow.Y_act, TrackFollow.delta);

             ControlClaw(ClawPos);
         end;
         2: begin
             form1.Memo1.Append('State 2');
             Speed:=PosControl(TrackFollow.X_act,TrackFollow.Y_act,FinalPose.Th,TrackFollow.delta);

             ControlClaw(ClawPos);
         end;
     else
         Speed.V:=0;
         Speed.Vn:=0;
         Speed.W:=0;
     end;

     SendVelServMessage(0);
end;

procedure TForm1.UDPTaskReceive(aSocket: TLSocket);
begin
  RecieveTaskMessage(0);
end;

procedure TForm1.Width_mapKeyPress(Sender: TObject; var Key: char);
begin
  if not (Key in ['0'..'9', '.','-', #8, #9]) then Key := #0;
end;




procedure TForm1.Height_mapKeyPress(Sender: TObject; var Key: char);
begin
  if not (Key in ['0'..'9', '.','-', #8, #9]) then Key := #0;
end;



end.

