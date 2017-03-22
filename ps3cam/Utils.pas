unit Utils;

interface

uses Classes, Graphics, sysutils, StdCtrls, Math;

type
  TMVEst=record
    Vmean,Vcov: double;
    speed: double;
    n: integer;
  end;

  TLinearReg=record
    Sx, Sy, Sxy, Sx2: double;
    a, b: double;
    mx, my: double;
    N: integer;
  end;


procedure ClearLinearReg(var TL: TLinearReg; saveMean: boolean = false);
procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
procedure CalcLinearReg(var TL: TLinearReg);

procedure MVEstInit(var MV: TMVEst);
procedure MVEstAddValue(var MV: TMVEst; v: double);

{$IFDEF UNIX}
procedure ZeroMemory(Ptr: Pointer; Len: integer);
procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);
//function GetTickCount: LongWord;
{$ENDIF}

function CalcCheckSum(const msg: string): byte;
function EditToFloatDef(edit: TEdit; default: double): double;
procedure ParseString(s,sep: string; sl: TStrings);

function FMod(const x,d: double): double;
function DiffAngle(const a1,a2: double): double;
function AngleThreePoints(const x1,y1,x2,y2,x3,y3: double): double;
function Dist(const x,y: double): double;
function DistPoints(const x1,y1, x2,y2: double): double;
function DistPoints2(const x1,y1, x2,y2: double): double;

// Point: x0,y0  Line: x1,y1 -> x2,y2
function DistPointLine(const x0,y0, x1,y1, x2,y2: double): double;

function ATan2(const y,x: double): double;
function Sign(const a: double): double;
function Sat(const a,limit: double): double;

function IncWrap(var v: integer; size: integer; step:integer=1): integer;
function DecWrap(var v: integer; size: integer; step:integer=1): integer;
procedure SwapInts(var v1, v2: integer);

//procedure OptimalMean(a,cov_a, b,cov_b: double; var m,cov_m: double);

procedure TranslateAndRotate(var rx,ry: double; const px,py,tx,ty,teta: double);
procedure TranslateAndRotateInv(var rx,ry: double; const px,py,tx,ty,teta: double);
procedure RotateAndTranslate(var rx,ry: double; const px,py,tx,ty,teta: double);

function InternalProductCosine(const v1x,v1y,v2x,v2y: double): double;
function NormalizeAngle(const ang: double): double;
function AverageAngle(const ang1,ang2: double): double;

function InFrustum(const xc,yc,xp,yp,ang,widthAng: double): boolean;




implementation

var FirstTimeValSec: LongInt;

function CalcCheckSum(const msg: string): byte;
var
i,check: byte;
begin
	check := 0;
  for i:=1 to length(msg) do begin
    check := check + ord(msg[i]);
	end;
	result:= check;
end;


{  intersection of the two infinite lines rather than the line segments }
procedure LinesIntersect(const x1,y1,x2,y2: double; { first line}
                         const x3,y3,x4,y4: double; { second line }
                         var code : integer; { =0 OK; =1 lines parallel}
                         var x,y : double); { intersection point }

var
    a1, a2, b1, b2, c1, c2 : double; { Coefficients of line eqns.}
    denom : double;

begin
  a1:= y2-y1;
  b1:= x1-x2;
  c1:= x2*y1 - x1*y2;  { a1*x + b1*y + c1 = 0 is line 1 }

  a2:= y4-y3;
  b2:= x3-x4;
  c2:= x4*y3 - x3*y4;  { a2*x + b2*y + c2 = 0 is line 2 }

  denom:= a1*b2 - a2*b1;
  if denom = 0 then
    begin
      code:=1;
      exit;
    end;

  x:=(b1*c2 - b2*c1)/denom;
  y:=(a2*c1 - a1*c2)/denom;
  code:=0
end;



function EditToFloatDef(edit: TEdit; default: double): double;
begin
  if edit.text='*' then begin
    result:=default;
    edit.text:=Format('%.8g',[default]);
    exit;
  end;
  try
    result:=strtofloat(edit.text);
  except
    result:=default;
    edit.text:=Format('%.8g',[default]);
  end;
end;


procedure ParseString(s,sep: string; sl: TStrings);
var p,i,last: integer;
begin
  sl.Clear;
  last:=1;
  for i:=1 to length(s) do begin
    p:=Pos(s[i],sep);
    if p>0 then begin
      if i<>last then
        sl.add(copy(s,last,i-last));
      last:=i+1;
    end;
  end;
  if last<=length(s) then
    sl.add(copy(s,last,length(s)-last+1));
end;

// ---------------------------------------------------------
//     Math functions

function Dist(const x,y: double): double;
begin
  result:=sqrt(x*x+y*y);
end;

function DistPoints(const x1,y1, x2,y2: double): double;
begin
  result:=sqrt(sqr(x1 - x2) + sqr(y1 - y2));
end;

function DistPoints2(const x1,y1, x2,y2: double): double;
begin
  result := sqr(x1 - x2) + sqr(y1 - y2);
end;


// Point: x0,y0  Line: x1,y1 -> x2,y2
function DistPointLine(const x0,y0, x1,y1, x2,y2: double): double;
begin
  result := abs((x2 - x1) * (y1 - y0) -(x1 - x0) * (y2 - y1))/ sqrt(sqr(x2 - x1) + sqr(y2 - y1));
end;

function FMod(const x,d: double): double;
begin
  result:=Frac(x/d)*d;
end;



function AngleThreePoints(const x1,y1,x2,y2,x3,y3: double): double;
var
a,b: double;
begin
  a:= atan2(y3-y1,x3-x1); // TODO: better algorithm
  b:= atan2(y2-y1,x2-x1);

  result := diffangle(a,b);
end;




function DiffAngle(const a1,a2: double): double;
begin
  result:=a1-a2;
  if result<0 then begin
    result:=-FMod(-result,2*Pi);
    if result<-Pi then result:=result+2*Pi;
  end else begin
    result:=FMod(result,2*Pi);
    if result>Pi then result:=result-2*Pi;
  end;
end;


function ATan2(const y,x: double): double;
var ax,ay: double;
begin
  ax:=Abs(x);
  ay:=Abs(y);

  if (ax<1e-10) and (ay<1e-10) then begin;
    result:=0.0;
    exit;
  end;
  if ax>ay then begin
    if x<0 then begin
      result:=ArcTan(y/x)+pi;
      if result>pi then result:=result-2*pi;
    end else begin
      result:=ArcTan(y/x);
    end;
  end else begin
    if y<0 then begin
      result:=ArcTan(-x/y)-pi/2
    end else begin
      result:=ArcTan(-x/y)+pi/2;
    end;
  end;
end;

function Sign(const a: double): double;
begin
  if a<0 then result:=-1 else result:=1;
end;

function Sat(const a,limit: double): double;
begin
 result := a;
 if a >  limit then result :=  limit;
 if a < -limit then result := -limit;
end;


function IncWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  inc(v,step);
  if v>=size then v:=v-size;
  Result:=v;
end;

function DecWrap(var v: integer; size: integer; step:integer=1): integer;
begin
  dec(v,step);
  if v<0 then v:=v+size;
  Result:=v;
end;

procedure SwapInts(var v1, v2: integer);
var t: integer;
begin
  t:=v1;
  v1:=v2;
  v2:=t;
end;


function Rad(const xw: double):double;
begin
  result:=xw*(pi/180);
end;

function deg(const xw: double):double;
begin
  result:=xw*(180/pi);
end;


function InternalProductCosine(const v1x,v1y,v2x,v2y: double): double;
var d: double;
begin
  d:=dist(v1x,v1y)*dist(v2x,v2y);
  if abs(d)<1e-6 then result:=-1
  else result:=(v1x*v2x+v1y*v2y)/d;
end;


procedure TranslateAndRotate(var rx,ry: double; const px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta

  vx:=px+tx;
  vy:=py+ty;
  rx:=vx*cos(teta)-vy*sin(teta);
  ry:=vx*sin(teta)+vy*cos(teta);
end;


procedure TranslateAndRotateInv(var rx,ry: double; const px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Translacao do vector (px,py) segundo o vector (tx,ty)
// seguida de Rotacao do angulo teta
// com matriz inversa
  vx:=px+tx;
  vy:=py+ty;

  rx:=vx*cos(teta)+vy*sin(teta);
  ry:=vx*-sin(teta)+vy*cos(teta);

end;


procedure RotateAndTranslate(var rx,ry: double; const px,py,tx,ty,teta: double);
var vx,vy: double;
begin
// Rotacao do vector (px,py) do angulo teta seguida de
// Translacao segundo o vector (tx,ty)

  vx:=px*cos(teta)-py*sin(teta);
  vy:=px*sin(teta)+py*cos(teta);
  rx:=vx+tx;
  ry:=vy+ty;
end;

function NormalizeAngle(const ang: double): double;
var a: double;
begin
  a:=FMod(ang+Pi,2*Pi);
  if a<0 then result:=a+Pi
  else result:=a-Pi;
end;


function AverageAngle(const ang1,ang2: double): double;
begin
  result:=NormalizeAngle(ang1+DiffAngle(ang2,ang1)*0.5);
end;


function InFrustum(const xc,yc,xp,yp,ang,widthAng: double): boolean;
begin
  result:=2*abs(DiffAngle(atan2(yp-yc,xp-xc),ang))<widthAng;
end;




{$IFDEF UNIX}
procedure ZeroMemory(Ptr: Pointer; Len: integer);
begin
  FillChar(Ptr^,len,0);
end;


procedure CopyMemory(DPtr,SPtr: Pointer; Len: integer);
begin
  Move(SPtr^,DPtr^,len);
end;

(*
function GetTickCount: LongWord;
var tv: TTimeVal;
begin
  GetTimeOfDay(tv,nil);
  if FirstTimeValSec=0 then FirstTimeValSec:=tv.tv_sec;
  result:=(tv.tv_sec-FirstTimeValSec)*1000+(tv.tv_usec div 1000);
end;  *)
//}
{$ENDIF}


procedure MVEstInit(var MV: TMVEst);
begin
  with MV do begin
    Vmean:=0;
    Vcov:=0;
    speed:=0.95;
    n:=0;
  end;
end;

procedure MVEstAddValue(var MV: TMVEst; v: double);
begin
  with MV do begin
    Vmean:=Vmean*speed+v*(1-speed);
    Vcov:=Vcov*speed+sqr(Vmean-v)*(1-speed);
    inc(n);
  end;
end;


procedure ClearLinearReg(var TL: TLinearReg; saveMean: boolean);
begin
  with TL do begin
    Sx:=0;
    Sy:=0;
    Sxy:=0;
    Sx2:=0;
    N:=0;
    a:=0;
    b:=0;
    if not SaveMean then begin
      mx:=0;
      my:=0;
    end;
  end;
end;


procedure AddXYtoLinearReg(var TL: TLinearReg; x,y: double);
var x0, y0: double;
begin
  with TL do begin
    x0 := x - mx;
    y0 := y - my;

    Sx := Sx + x0;
    Sy := Sy + y0;

    Sxy:= Sxy + x0 * y0;
    Sx2:= Sx2 + x0 * x0;

    inc(N);
  end;
end;

procedure CalcLinearReg(var TL: TLinearReg);
var d: double;
begin
  with TL do begin
    a := 0;
    b := 0;
    if N = 0 then exit;
    mx := Sx / N;
    my := Sy / N;
    d:=N*Sx2-Sx*Sx;
    if abs(d)<1e-8 then exit;
    b:=(N*Sxy-Sx*Sy)/d;
    a:=my-b*mx;
  end;
end;

procedure SaveStrinGtoFile(Filename: string; data: string);
var fs: TFilestream;
begin
  fs := TFilestream.Create(Filename, fmCreate);
  try
    fs.WriteAnsiString(data);
  finally
   fs.Free;
  end;
end;

function LoadStrinGFromFile(Filename: string): string;
var fs: TFilestream;
begin
  fs := TFilestream.Create(Filename, fmOpenRead);
  try
    result := fs.ReadAnsiString;
  finally
   fs.Free;
  end;


end;

{procedure SaveStrinGtoFile(Filename: string; data: string);
var FD: Longint;
begin
  FD := FileCreate(Filename);
  if FD < 0 then
    raise Exception.CreateFmt('%Error when creating File: "%s"', [Filename]);
  if FileWrite(FD, data[1], Length(data)) <> length(data) then
      raise Exception.CreateFmt('%Error when writing File: "%s"', [Filename]);
  FileClose(FD);
end;

function LoadStrinGFromFile(Filename: string): string;
var FD, fs: Longint;
begin
  filestream
  fs := (Filename);
  FD := FileOpen(Filename, fmOpenRead);
  if FD < 0 then
    raise Exception.CreateFmt('%Error when opening File: "%s"', [Filename]);
  result := StringOfChar(#0, fs);
  if FileRead(FD, result[1], fs) <> fs then
      raise Exception.CreateFmt('%Error when reading File: "%s"', [Filename]);
  FileClose(FD);
end;
}

end.

