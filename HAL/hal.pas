unit hal;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, structs;

type

  { TFHal }

  TFHal = class(TForm)
    BSetMission: TButton;
    BStart: TButton;
    CBDebug: TCheckBox;
    EditMacFrom: TEdit;
    EditMacTo: TEdit;
    EditActionDebug: TLabeledEdit;
    EditYDebug: TLabeledEdit;
    EditThDebug: TLabeledEdit;
    debugBoxColor: TLabeledEdit;
    GBWareMac: TGroupBox;
    From: TLabel;
    EditXDebug: TLabeledEdit;
    Memo1: TMemo;
    Tolabel: TLabel;
    Part1: TEdit;
    GBParts: TGroupBox;
    inesc: TImage;
    Part2: TEdit;
    Part3: TEdit;
    Part4: TEdit;
    Part5: TEdit;
    RGMission: TRadioGroup;
    procedure BSetMissionClick(Sender: TObject);
    procedure BStartClick(Sender: TObject);
    procedure ChangePartColor(color_part:  array of TPart);
    procedure debugBoxColorChange(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure Memo1Change(Sender: TObject);
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
function TFHal.setcolor(PartType : TPartType; var partNumber: integer): TColor;
var partcolor:TColor;
begin

    if PartType = A then begin
      partcolor:= clRed;
      if ve = 1 then begin
            memo1.append('red');
            PartsScript[totalParts].PartType:= A;
            PartsScript[totalParts].ID:= totalParts;
            PartsScript[totalParts].Node:= BoxPlaces[totalParts];
            ve := 25;
      end
      else if  ((ve = 2) and (partNumber = 5)) then begin
           memo1.append('red');
           PartsScript[totalParts].PartType:= A;
           PartsScript[totalParts].ID:= totalParts;
           PartsScript[totalParts].Node:= BoxPlaces[totalParts];
           ve := 25;
      end;

   end else if PartType = B then begin
      partcolor:= clGreen;
            if ve = 1 then begin
            memo1.append('green');
            PartsScript[totalParts].PartType:= B;
            PartsScript[totalParts].ID:= totalParts;
            PartsScript[totalParts].Node:= BoxPlaces[totalParts];
            ve := 25;
            end
            else if  ((ve = 2) and (partNumber = 5)) then begin
                 memo1.append('green');
                 PartsScript[totalParts].PartType:= B;
                 PartsScript[totalParts].ID:= totalParts;
                 PartsScript[totalParts].Node:= BoxPlaces[totalParts];
                 ve := 25;
            end;
   end else if PartType = C then begin
      partcolor:= clBlue;
            if ve = 1 then begin
            memo1.append('blue');
            PartsScript[totalParts].PartType:= C;
            PartsScript[totalParts].ID:= totalParts;
            PartsScript[totalParts].Node:= BoxPlaces[totalParts];
            ve := 25;
            end
            else if  ((ve = 2) and (partNumber = 5)) then begin
                 memo1.append('blue');
                 PartsScript[totalParts].PartType:= C;
                 PartsScript[totalParts].ID:= totalParts;
                 PartsScript[totalParts].Node:= BoxPlaces[totalParts];
                 ve := 25;
            end;
   end else  begin
      partcolor:= clBlack;
   end;
   Result:=partcolor;
end;

procedure TFHal.ChangePartColor(color_part: array of TPart);
var partN: integer;
begin
    partN:=1;
    color_part[1].PartType:=A;
    Part1.Color := setcolor(color_part[1].PartType, partN);
    partN:=2;
    color_part[2].PartType:=C;
    Part2.Color := setcolor(color_part[2].PartType, partN);
    partN:=3;
    color_part[3].PartType:=B;
    Part3.Color := setcolor(color_part[3].PartType, partN);
    partN:=4;
    color_part[4].PartType:=B;
    Part4.Color := setcolor(color_part[4].PartType, partN);
    partN:=5;
    color_part[5].PartType:=C;
    Part5.Color := setcolor(color_part[5].PartType, partN);
end;

procedure TFHal.debugBoxColorChange(Sender: TObject);
begin

end;

procedure TFHal.FormShow(Sender: TObject);
begin
   Part1.Color := clBlack;
   Part2.Color := clBlack;
   Part3.Color := clBlack;
   Part4.Color := clBlack;
   Part5.Color := clBlack;
end;

procedure TFHal.Memo1Change(Sender: TObject);
begin

end;

procedure TFHal.BSetMissionClick(Sender: TObject);
begin
  (*Pose_loc.x := BoxPlaces[18].x;
  Pose_loc.y := BoxPlaces[18].y;
  Pose_loc.theta := BoxPlaces[18].teta;
  RobotState.servo1 := 0;
  RobotState.servo2 := 0;

  memo1.Clear;
  ve := 0;
  rebuild := 0;
  BuildStateMachine(true);
  ResetStateMachine();  *)

   memo1.Clear;
   memo1.Append('red');
   memo1.Append('blue');
   memo1.Append('green');
   memo1.Append('green');
   memo1.Append('blue');

   PartsScript[1].PartType:= A;
                 PartsScript[1].ID:= 1;
                 PartsScript[1].Node:= BoxPlaces[1];

   ChangePartColor(PartsScript);
end;

procedure TFHal.BStartClick(Sender: TObject);
begin
  FLaserLoc.CBSendLock.Checked := not(FLaserLoc.CBSendLock.Checked);
  init  := 0;
end;


initialization
  {$I hal.lrs}

end.

