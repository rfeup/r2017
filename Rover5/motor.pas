unit motor;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, StdCtrls, math;

type

  TEncoder = record
    Position, LastPosition: integer;
    ReadTime, LastReadTime: DWord;
    TickCount: integer;
    speed: double;
    KEnc: double;
  end;


  TPID = record
    Ki, Kd, Kp, Kf: double;
    Sek, ek_1, ek: double;
    m_sat: double;
    Ticks, ControlPeriod: double;
  end;


  TMotGUI = record
    SBM: TScrollBar;
    EditM: TEdit;
  end;

  { TMotor }

  TMotor = class
  private

  public
    RefPWM: integer;
    ReadPWM: integer;
    RefU, ReadU: double;

    ReadIm: integer;
    Im: double;

    Encoder: TEncoder;

    PID: TPID;

    constructor Create;
    destructor Destroy; override;
  end;

function CalcPID(var PID: TPID; ref, yk: double): double;


implementation

function CalcPID(var PID: TPID; ref, yk: double): double;
var ek, dek, mk: double;
begin
  result := 0;

  ek := ref - yk;
  dek := ek - PID.ek_1;
  PID.Sek := PID.Sek + ek;
  mk := PID.Kp * ek + PID.Ki * PID.Sek + PID.Kd * dek + PID.Kf * ref;

  // Anti Windup
  if abs(mk) >= PID.m_sat then begin
    PID.Sek := PID.Sek - ek;
    mk := max(-PID.m_sat, min(PID.m_sat, mk));
  end;

  PID.ek_1 := ek;
  result := mk;

  if ek = 0 then
    PID.Sek := PID.Sek * 0.97;
end;


{ TMotor }

constructor TMotor.Create;
begin

end;

destructor TMotor.Destroy;
begin
  inherited Destroy;
end;

end.
