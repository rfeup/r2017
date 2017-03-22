unit PathFollowing;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Math, dynmatrix, lNet, Utils;

type
TRobotControls = record
  V, Vn, W: double;
end;
TLine = record
  Xi, Yi, Xf, Yf: double;
end;

TPose = record
  X, Y, Th: double;
end;

DoubleArray = array of Double;
TLineArray = array of TLine;

TDataPath = record
    Xi, X_act, Y_act, delta: double;
end;

const
  K1 = 1.5;//2.75;
  K2 = 3; //2.5;//3.25;
  K3 = 2;// x_act correction control pos
  K4 = 2;// y_act correction control pos
  K5 = 3.5; // rotation control pos
  K6 = 1.5; // y_act correction goto
  K7 = 0.3; // rotation goto
  K8 = 1; // X_act correction goto
  d1 = 0.05;
  d2 = 0.08;
  FMaxDistance_Error = 0.005;
  FMaxAngle_Error = 2 * pi/180;
  MAX_GOTO_VEL = 0.15;

Var
   Y_loc, X_loc, Th_loc:double;
   Vnomin, Vfinal, Th_nomin, Th_final:double;
   FMover_xy : Integer;


function NormalAng(ang: double) : double;
function RotMat(x, y, angle : Double) : DoubleArray;
function PosControl(X, Y, Th, delta: Double) : TRobotControls;
function FollowLine(Xi, X_act, Y_act, delta: double) : TRobotControls;
function ConfigTrack(Line : TLine):TDataPath;
function DecompPath(Dist : TDMatrix; Path : TLine; Robot_r, Sizec:Double): TLineArray;
function GoToXYTheta(Path : TDataPath; WantedPose:TPose; velocity: double): TRobotControls;

implementation

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

function RotMat(x, y, angle : Double) : DoubleArray;
var
  pos : DoubleArray;
begin
   SetLength(pos,2);

   pos[0] := x*cos(angle) + y*sin(angle);
   pos[1] := -x*sin(angle) + y*cos(angle);

   result := pos;
end;

function PosControl(X, Y, Th, delta: Double) : TRobotControls;
var
   Robot_vel : TRobotControls;
   AuxVel : DoubleArray;
   Va, Vc, eth: Double;
begin
    Va := -K3*X;
    Vc := -K4*Y;

    AuxVel := RotMat(Va, Vc, delta);

    Robot_vel.V  := AuxVel[0];
    Robot_vel.Vn := AuxVel[1];

    eth := NormalAng(DiffAngle(Th, Th_loc));

    Robot_vel.W  := eth*k5;

    Result:=Robot_vel;
end;

function FollowLine(Xi, X_act, Y_act, delta: double) : TRobotControls;
var
   d : Double;
   Vc, Va, Vr, es, eth, Th_ref: Double;
   AuxPos, AuxVel : DoubleArray;
   Robot_vel : TRobotControls;
begin

   d := abs(X_act/Xi);
   Vr := (Vnomin*(d) + Vfinal*(1-d));

   if (X_act < Xi+d1) and (Xi+d1 < -d2) then begin
     if  X_act < Xi then begin
         Vr := Vnomin*(0.2);
     end else begin
         Vr := Vnomin*0.6/(d1)*X_act+(0.4*Vnomin*(Xi+d1)-Vnomin*Xi)/(d1);
     end;
   end else if (X_act >= Xi+d1) and (X_act <= -d2) then begin
     Vr := Vnomin;
   end else begin
     Vr := (Vfinal-Vnomin)/(d2)*X_act + Vfinal;
   end;


   Vc := -Y_act * k1;

   if (Vr*Vr - Vc*Vc) <= 0 then begin
       Va := 0;
   end;
   if (Vr*Vr - Vc*Vc) > 0   then begin
       Va := sqrt(Vr*Vr - Vc*Vc);
   end;

   AuxVel := RotMat(Va, Vc, delta);

   Robot_vel.V  := AuxVel[0];
   Robot_vel.Vn := AuxVel[1];

   d := abs(X_act/Xi);

   Th_ref := NormalAng(DiffAngle(Th_nomin*(d), -Th_final*(1-d)));

   eth := NormalAng(DiffAngle(Th_ref, Th_loc));

   Robot_vel.W  := eth*k2;

   Result := Robot_vel;
end;

function ConfigTrack(Line : TLine):TDataPath;
var
   delta, alpha: Double;
   AuxPos : DoubleArray;
   xi, xf, X_act, yi, yf, Y_act : Double;
begin
     alpha := NormalAng(arctan2(Line.yf-Line.yi,Line.xf-Line.xi));
     delta := NormalAng(DiffAngle(Th_loc,alpha));


     AuxPos := RotMat(Line.xf, Line.yf, alpha);
     xf:=AuxPos[0]; yf:=AuxPos[1];

     AuxPos := RotMat(Line.xi, Line.yi, alpha);
     xi:=AuxPos[0]; yi:=AuxPos[1];

     AuxPos := RotMat(X_loc, Y_loc, alpha);

     X_act:=AuxPos[0]-xf; Y_act:=AuxPos[1]-yf;

     xi:=xi-xf; yi:=yi-yf;
     xf:= 0; yf:=0;

     Result.Xi:=xi;
     Result.X_act:=X_act;
     Result.Y_act:=Y_act;
     result.delta:=delta;

end;

function DecompPath(Dist : TDMatrix; Path : TLine; Robot_r, Sizec:Double): TLineArray;
var
   Row, Col, IniX, IniY, FinX, FinY, i : integer;
   Aux, Aux2 : Double;
   Path_dec : TLineArray;

begin
     IniX:= round((Path.Xi-Sizec/2)/Sizec);
     IniY:= round((Path.Yi-Sizec/2)/Sizec);
     FinX:= round((Path.Xf-Sizec/2)/Sizec);
     FinY:= round((Path.Yf-Sizec/2)/Sizec);
     SetLength(Path_dec, 3);

     i:=0;
     Aux2 := 0;

     if Dist.ugetv(IniY,IniX) > Robot_r then begin
         // Return X e Y caminho
         Path_dec[0].Xi:=Path.Xi;
         Path_dec[0].Yi:=Path.Yi;
         Path_dec[0].Xf:=Path.Xi;
         Path_dec[0].Yf:=Path.Yi;
     end else begin
         Path_dec[0].Xi:=Path.Xi;
         Path_dec[0].Yi:=Path.Yi;

         for Row := round(IniY - Robot_r/Sizec) to round(IniY + Robot_r/Sizec) do begin
         for Col := round(IniX - Robot_r/Sizec) to round(IniX + Robot_r/Sizec) do begin
              if (Dist.ugetv(Row,Col) > Robot_r) and (Dist.ugetv(Row,Col) <> 0)then begin
                  Aux :=Dist.ugetv(Row,Col);
                  if Aux > Aux2 then begin
                      Aux2 := Aux;
                      Path_dec[0].Xf:=Sizec/2 + Sizec*Col;
                      Path_dec[0].Yf:=Sizec/2 + Sizec*Row;
                  end;
              end;
         end;
         end;
     end;

     i:=0;
     Aux2 := 0;

     if Dist.ugetv(FinY,FinX) > Robot_r then begin
         // Return X e Y caminho
         Path_dec[2].Xi:=Path.Xf;
         Path_dec[2].Yi:=Path.Yf;
         Path_dec[2].Xf:=Path.Xf;
         Path_dec[2].Yf:=Path.Yf;
     end else begin
         Path_dec[2].Xf:=Path.Xf;
         Path_dec[2].Yf:=Path.Yf;

         for Row := round(FinY - Robot_r/Sizec) to round(FinY + Robot_r/Sizec) do begin
         for Col := round(FinX - Robot_r/Sizec) to round(FinX+ Robot_r/Sizec) do begin
              if (Dist.ugetv(Row,Col) > Robot_r) and (Dist.ugetv(Row,Col) <> 0)then begin
                  Aux :=Dist.ugetv(Row,Col);
                  if Aux > Aux2 then begin
                      Aux2 := Aux;
                      Path_dec[2].Xi:=Sizec/2 + Sizec*Col;
                      Path_dec[2].Yi:=Sizec/2 + Sizec*Row;
                  end;
              end;
         end;
         end;
     end;

     Path_dec[1].Xi:=Path_dec[0].Xf;
     Path_dec[1].Yi:=Path_dec[0].Yf;
     Path_dec[1].Xf:=Path_dec[2].Xi;
     Path_dec[1].Yf:=Path_dec[2].Yi;

     Result := Path_dec;
end;


function GoToXYTheta(Path : TDataPath; WantedPose:TPose; velocity: double): TRobotControls;
var
  Robot_vel : TRobotControls;
  alpha, distance:double;
  AuxVel : DoubleArray;
  Va, Vc, eth: Double;
begin

 distance:= Dist(Path.X_act, Path.Y_act);
 alpha:= Path.delta;

 case FMover_xy of
   1: //STOP
      begin
        Robot_vel.V := 0 ;
        Robot_vel.Vn := 0 ;
        Robot_vel.W := 0 ;

        if distance <= FMaxDistance_Error then   begin
           FMover_xy:= 4;
        end else begin
           FMover_xy:=2;
        end;
      end;


   2: // Moving and rotating - PHASE 1             max vel movement
      begin
        Va := -Sign(Path.X_act) * MAX_GOTO_VEL;

        if Path.Y_act < 0.03 then begin
           Vc := -K6*Path.Y_act;
           //Vc := -Sign(Path.Y_act) * MAX_GOTO_VEL/2;
        end else  begin
           Vc := -Sign(Path.Y_act) * MAX_GOTO_VEL/2;
        end;

        AuxVel := RotMat(Va, Vc, Path.delta);

        Robot_vel.V  := AuxVel[0];
        Robot_vel.Vn := AuxVel[1];

        eth          := (*arctan2(WantedPose.Y-Y_loc,WantedPose.X-X_loc*)WantedPose.Th- Th_loc;
        Robot_vel.W  := eth*k7;

        //eth :=NormalAng(DiffAngle(WantedPose.Th, Th_loc));//NormalAng(arctan2(sin(DiffAngle(WantedPose.Th, Th_loc)), cos(DiffAngle(WantedPose.Th, Th_loc))));   //
        //Are we there yet?
        if (distance <= 2*FMaxDistance_Error)  then begin
           FMover_xy:= 3;
        end else begin
           FMover_xy:= 2;
        end;
      end;

   3: // Moving and rotating - PHASE 2             Approach vel movement
      begin
        Va := -K3*Path.X_act;
        Vc := -K4*Path.Y_act;

        AuxVel := RotMat(Va, Vc, Path.delta);

        Robot_vel.V  := AuxVel[0];
        Robot_vel.Vn := AuxVel[1];

        eth := NormalAng(DiffAngle(WantedPose.Th, Th_loc));

        Robot_vel.W  := eth*K5;

        //Are we there yet?
        if (distance<= FMaxDistance_Error)  then begin FMover_xy:= 1; end
        else begin FMover_xy:= 3; end;
      end;

   4: // FINAL ORIENTATION
      begin
        Va := -K3*Path.X_act;
        Vc := -K4*Path.Y_act;

        AuxVel := RotMat(Va, Vc, Path.delta);

        Robot_vel.V  := AuxVel[0];
        Robot_vel.Vn := AuxVel[1];

        eth := NormalAng(DiffAngle(WantedPose.Th, Th_loc));

        Robot_vel.W  := eth*K5;

        //Are we pointing the right way??
        if (abs(eth) < FMaxAngle_Error) then begin FMover_xy:=1; end
        else begin FMover_xy:=4; end;
      end;
 end;
 Result:=Robot_vel;
end;
end.

