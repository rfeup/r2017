unit hal;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, IniPropStorage, structs;

type

  { TFHal }

  TFHal = class(TForm)
    BSetMission: TButton;
    EDefaultSpeed: TEdit;
    EApproachSpeed: TEdit;
    EMission: TEdit;
    EPathRadius: TEdit;
    GBVariables: TGroupBox;
    IniPropStorage: TIniPropStorage;
    LDefaultSpeed: TLabel;
    LApproachSpeed: TLabel;
    LPathRadius: TLabel;
    RGOperationMode: TRadioGroup;
    BStart: TButton;
    CBDebug: TCheckBox;
    EditMacFrom: TEdit;
    EditMacTo: TEdit;
    EActionDebug: TLabeledEdit;
    EditYDebug: TLabeledEdit;
    EditThDebug: TLabeledEdit;
    debugBoxColor: TLabeledEdit;
    GBWareMac: TGroupBox;
    From: TLabel;
    EditXDebug: TLabeledEdit;
    DebugMenu: TMemo;
    Tolabel: TLabel;
    GBParts: TGroupBox;
    inesc: TImage;
    Part1: TEdit;
    Part2: TEdit;
    Part3: TEdit;
    Part4: TEdit;
    Part5: TEdit;
    RGMission: TRadioGroup;
    procedure BSetMissionClick(Sender: TObject);
    procedure BStartClick(Sender: TObject);
    procedure ChangePartColor(color_part:  array of TPart);
    procedure FormShow(Sender: TObject);


    function setcolor(PartType : TPartType; var partNumber: integer): TColor;
  private
    { private declarations }
  public
    { public declarations }
  end;


var
  FHal: TFHal;

implementation

uses state, laserloc;

{------------------------------------------------------------------------------
       TFHal.SetColor
------------------------------------------------------------------------------}

function TFHal.setcolor(PartType : TPartType; var partNumber: integer): TColor;
var partcolor:TColor;
begin

    if PartType = R then begin
      partcolor:= clRed;

            DebugMenu.append('red');
            PartsScript[partNumber].PartType:= R;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];


      //end;

   end else if PartType = G then begin
      partcolor:= clGreen;
      //if ve = 1 then begin
      //      DebugMenu.append('green');
      //      PartsScript[totalParts].PartType:= G;
      //      PartsScript[totalParts].ID:= totalParts;
      //      PartsScript[totalParts].Node:= BoxPlaces[totalParts];
      //      ve := 25;
      //end
      //else if  ((ve = 2) and (partNumber = 5)) then begin
            DebugMenu.append('green');
            PartsScript[partNumber].PartType:= G;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];
      //      ve := 25;
      //end;
   end else if PartType = B then begin
      partcolor:= clBlue;
      //if ve = 1 then begin
            DebugMenu.append('blue');
            PartsScript[partNumber].PartType:= B;
            PartsScript[partNumber].ID:= partNumber;
            PartsScript[partNumber].Node:= BoxPlaces[partNumber];
            ve := 25;
      //end
      //else if  ((ve = 2) and (partNumber = 5)) then begin
      //      DebugMenu.append('blue');
      //      PartsScript[totalParts].PartType:= B;
      //      PartsScript[totalParts].ID:= totalParts;
      //      PartsScript[totalParts].Node:= BoxPlaces[totalParts];
      //      ve := 25;
      //end;
   end else begin
      partcolor:= clBlack;
   end;
   Result:=partcolor;
end;

{------------------------------------------------------------------------------
       TFHal.ChangePartColor
------------------------------------------------------------------------------}

procedure TFHal.ChangePartColor(color_part: array of TPart);
var partN: integer;
begin
    partN:=1;
    color_part[1].PartType:=B;
    Part1.Color := setcolor(color_part[1].PartType, partN);
    partN:=2;
    color_part[2].PartType:=R;
    Part2.Color := setcolor(color_part[2].PartType, partN);
    partN:=3;
    color_part[3].PartType:=G;
    Part3.Color := setcolor(color_part[3].PartType, partN);
    partN:=4;
    color_part[4].PartType:=G;
    Part4.Color := setcolor(color_part[4].PartType, partN);
    partN:=5;
    color_part[5].PartType:=B;
    Part5.Color := setcolor(color_part[5].PartType, partN);
end;

{------------------------------------------------------------------------------
       TFHal.FormShow
------------------------------------------------------------------------------}

procedure TFHal.FormShow(Sender: TObject);
begin
   Part1.Color := clBlack;
   Part2.Color := clBlack;
   Part3.Color := clBlack;
   Part4.Color := clBlack;
   Part5.Color := clBlack;
end;







{------------------------------------------------------------------------------
       TFHal.BSetMissionClick
------------------------------------------------------------------------------}

procedure TFHal.BSetMissionClick(Sender: TObject);
var missao: integer;
begin

  Pose_loc.x := BoxPlaces[18].x;
  Pose_loc.y := BoxPlaces[18].y;
  Pose_loc.theta := BoxPlaces[18].teta;
  RobotState.servo1 := 0;
  RobotState.servo2 := 0;

  DebugMenu.Clear;
  ve := 0;
  rebuild := 0;

  case RGOperationMode.ItemIndex of
       0 : BuildStateMachine(false, false);
       1 : BuildStateMachine(true, false);
       2 : BuildStateMachine(false, true);
  end;

  ResetStateMachine();

   (*DebugMenu.Clear;
   DebugMenu.Append('red');
   DebugMenu.Append('blue');
   DebugMenu.Append('green');
   DebugMenu.Append('green');
   DebugMenu.Append('blue');*)



end;

{------------------------------------------------------------------------------
       TFHal.StartButtonClick
------------------------------------------------------------------------------}

procedure TFHal.BStartClick(Sender: TObject);
begin

  FLaserLoc.CBSendLock.Checked := not(FLaserLoc.CBSendLock.Checked);
  init  := 0;

end;


initialization
  {$I hal.lrs}

end.

