unit CameraUDP;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, structs, lNetComponents,forms, state, rlan, hal;

var
MsgMachineState : TStringArray;
function  UpdatePartsFromCamera(Camcolor : string): TPartType;
procedure UDPCameraReceivePart(MessaC:String);
procedure PathSelectionMission1();
procedure PathSelectionMission2();
procedure PathSelectionMission3();

implementation

uses laserloc;

function  UpdatePartsFromCamera(Camcolor : string): TPartType;
begin
  case Camcolor of
       'r': result:= R;
       'g': result:= G;
       'b': result:= B;
  else
      result:=N;
  end;
end;

procedure PathSelectionMission1();
begin
  path_1[0, 0] := 0; path_1[0, 1] := 5;
  path_1[1, 0] := 1; path_1[1, 1] := 6;
  path_1[2, 0] := 2; path_1[2, 1] := 7;
  path_1[3, 0] := 3; path_1[3, 1] := 8;
  path_1[4, 0] := 4; path_1[4, 1] := 9;
end;

procedure PathSelectionMission2();
begin
  if PartsScript[2].PartType = G then begin  {B | G | B | B | G}   //2.35
    path_2[0, 0] := 0; path_2[0, 1] := 9;
    path_2[1, 0] := 1; path_2[1, 1] := 16;
    path_2[2, 0] := 4; path_2[2, 1] := 14;
    path_2[3, 0] := 2; path_2[3, 1] := 8;
    path_2[4, 0] := 3; path_2[4, 1] := 7;
    path_2[5, 0] := 16; path_2[5, 1] := 6;
    path_2[6, 0] := 14; path_2[6, 1] := 5;
  end else if PartsScript[3].PartType = G then begin {B | B | G | B | G}   //2.22
    path_2[0, 0] := 0; path_2[0, 1] := 9;
    path_2[1, 0] := 2; path_2[1, 1] := 16;
    path_2[2, 0] := 4; path_2[2, 1] := 14;
    path_2[3, 0] := 1; path_2[3, 1] := 8;
    path_2[4, 0] := 3; path_2[4, 1] := 7;
    path_2[5, 0] := 16; path_2[5, 1] := 6;
    path_2[6, 0] := 14; path_2[6, 1] := 5;
  end else if PartsScript[4].PartType = G then begin {B | B | B | G | G}   //2.11
    path_2[0, 0] := 0; path_2[0, 1] := 9;
    path_2[1, 0] := 3; path_2[1, 1] := 16;
    path_2[2, 0] := 4; path_2[2, 1] := 14;
    path_2[3, 0] := 1; path_2[3, 1] := 8;
    path_2[4, 0] := 2; path_2[4, 1] := 7;
    path_2[5, 0] := 16; path_2[5, 1] := 6;
    path_2[6, 0] := 14; path_2[6, 1] := 5;
  end else begin
    rebuild := 1;
  end;

end;

procedure PathSelectionMission3();
begin

  if (PartsScript[2].PartType = B) and (PartsScript[5].PartType = R) then begin  {B | B | G | G | R}      //2.42
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 2; path_3[1, 1] := 14;
    path_3[2, 0] := 4; path_3[2, 1] := 11;
    path_3[3, 0] := 3; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 1; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[2].PartType = B) and (PartsScript[4].PartType = R) then begin  {B | B | G | R | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 2; path_3[1, 1] := 14;
    path_3[2, 0] := 3; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 1; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[2].PartType = B) and (PartsScript[3].PartType = R) then begin  {B | B | R | G | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 3; path_3[1, 1] := 14;
    path_3[2, 0] := 2; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 1; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[5].PartType = B) and (PartsScript[4].PartType = R) then begin  {B | G | G | R | B}   //   20539
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 3; path_3[2, 1] := 11;
    path_3[3, 0] := 2; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 4; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[4].PartType = B) and (PartsScript[5].PartType = R) then begin  {B | G | G | B | R}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 4; path_3[2, 1] := 11;
    path_3[3, 0] := 2; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 3; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[4].PartType = B) and (PartsScript[3].PartType = R) then begin  {B | G | R | B | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 2; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 3; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;


  end else if (PartsScript[5].PartType = B) and (PartsScript[3].PartType = R) then begin  {B | G | R | G | B}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 2; path_3[2, 1] := 11;
    path_3[3, 0] := 3; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 4; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[3].PartType = B) and (PartsScript[4].PartType = R) then begin  {B | G | B | R | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 3; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 2; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[3].PartType = B) and (PartsScript[5].PartType = R) then begin  {B | G | B | G | R}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 1; path_3[1, 1] := 14;
    path_3[2, 0] := 4; path_3[2, 1] := 11;
    path_3[3, 0] := 3; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 2; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;



  end else if (PartsScript[3].PartType = B) and (PartsScript[2].PartType = R) then begin  {B | R | B | G | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 3; path_3[1, 1] := 14;
    path_3[2, 0] := 1; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 2; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[4].PartType = B) and (PartsScript[2].PartType = R) then begin  {B | R | G | B | G}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 2; path_3[1, 1] := 14;
    path_3[2, 0] := 1; path_3[2, 1] := 11;
    path_3[3, 0] := 4; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 3; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;

  end else if (PartsScript[5].PartType = B) and (PartsScript[2].PartType = R) then begin  {B | R | G | G | B}
    path_3[0, 0] := 0; path_3[0, 1] := 9;
    path_3[1, 0] := 3; path_3[1, 1] := 14;
    path_3[2, 0] := 1; path_3[2, 1] := 11;
    path_3[3, 0] := 2; path_3[3, 1] := 16;
    path_3[4, 0] := 14; path_3[4, 1] := 8;
    path_3[5, 0] := 11; path_3[5, 1] := 14;
    path_3[6, 0] := 4; path_3[6, 1] := 5;
    path_3[7, 0] := 16; path_3[7, 1] := 6;
    path_3[8, 0] := 14; path_3[8, 1] := 7;
  end else begin
      rebuild := 1;
    end
  end;

procedure UDPCameraReceivePart(MessaC:String);
    var i: integer;
        last_drop2: integer;
        CrossColorCheck_1, CrossColorCheck_2 : TPartType;
    begin
      // Parts State - check parts poses

  PartsScript2[1].PartType:= UpdatePartsFromCamera(MsgMachineState[1]);
  PartsScript2[1].ID:= 1;
  PartsScript2[1].Node:= BoxPlaces[0];
  PartsScript2[2].PartType:= UpdatePartsFromCamera(MsgMachineState[2]);
  PartsScript2[2].ID:= 2;
  PartsScript2[2].Node:= BoxPlaces[1];
  PartsScript2[3].PartType:= UpdatePartsFromCamera(MsgMachineState[3]);
  PartsScript2[3].ID:= 3;
  PartsScript2[3].Node:= BoxPlaces[2];
  PartsScript2[4].PartType:= UpdatePartsFromCamera(MsgMachineState[4]);
  PartsScript2[4].ID:= 4;
  PartsScript2[4].Node:= BoxPlaces[3];
  PartsScript2[5].PartType:= UpdatePartsFromCamera(MsgMachineState[5]);
  PartsScript2[5].ID:= 5;
  PartsScript2[5].Node:= BoxPlaces[4];


     FHal.ChangePartColor(PartsScript2);


   end;

end.

