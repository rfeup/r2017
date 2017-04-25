unit structs;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

const

  NUM_Parts = 5;
  maxActions = 4096;

type

  TStringArray= array [1 ..5] of string;

  TIntMatrix = array of array of integer;

    {*************** Actions  ********************}

  TactionType = (atGoToXY, atSwivelServo, atRotateAndGo, atGoToWarehouse, atFollowArc, atStop, atWait, atCheck, atConfirm, atStabilize);

  Taction = record
    atype      : TactionType;
    x,y,teta   : double;
    path_teta  : double;
    speed      : double;
    final_speed: double;

    path_length, teta_start: double;
    radius, rteta_start, rteta_end: double;

    reset_x, reset_y: double;
    reset_coords: boolean;
  end;

  TBoxPlaceKind = (bpkSource, bpkTarget, bpkMachine, bpkStart);
  TBoxPlace = record
    x, y, teta, advance: double;
    kind               : TBoxPlaceKind;
  end;

    {*************** Parts  ********************}

  TPartType = (R, G, B, N);


  TPart = record
    ID       : integer; // [1 .. 5]
    PartType : TPartType;
    Node     : TBoxPlace;
  end;

  {*************** Robot Properties  ********************}

  TRobotState = record
    x, y, teta    : double;
    err           : double;
    v, vn, w     : double;
    servo1, servo2: double;

    last_v, last_vn, last_w: double;
  end;

  TIntegerMatrix1 = Array[0..4,0..1] of Integer; { Mission 1 }
  TIntegerMatrix2 = Array[0..6,0..1] of Integer; { Mission 2 }
  TIntegerMatrix3 = Array[0..8,0..1] of Integer; { Mission 3 }

  var

    path_1 : TIntegerMatrix1 = (
         (0, 5),
         (1, 6),
         (2, 7),
         (3, 8),
         (4, 9)
    );

    path_2 : TIntegerMatrix2;
    path_3 : TIntegerMatrix3;

implementation


end.

