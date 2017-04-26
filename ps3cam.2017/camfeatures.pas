unit camfeatures;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LResources, Forms, Controls, Graphics, Dialogs,
  Grids, StdCtrls, IniPropStorage, SdpoVideo4L2, VideoDev2;

type

  { TFCamFeatures }

  TFCamFeatures = class(TForm)
    BFeatureValueSet: TButton;
    BSave: TButton;
    BLoad: TButton;
    EditFeatureValue: TEdit;
    IniPropStorage: TIniPropStorage;
    LabelFeature: TLabel;
    SBFeatureValue: TScrollBar;
    SGVideoControls: TStringGrid;
    procedure BFeatureValueSetClick(Sender: TObject);
    procedure BLoadClick(Sender: TObject);
    procedure BSaveClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure SBFeatureValueChange(Sender: TObject);
    procedure SGVideoControlsSelectCell(Sender: TObject; aCol, aRow: Integer;
      var CanSelect: Boolean);
  private
    procedure RefreshFeatures;
    function SafeGetFeatureValue(i: LongWord): LongWord;
    procedure SetCurFeatureValue(newvalue: dword);
    { private declarations }
  public
    Video: TSdpoVideo4L2;
    curFeatureIdx: integer;
    updatingHeader: integer;
  end;

var
  FCamFeatures: TFCamFeatures;

implementation

uses main;

{ TFCamFeatures }

function TFCamFeatures.SafeGetFeatureValue(i: LongWord): LongWord;
begin
  if (Video.UserControls[i].flags and (V4L2_CTRL_FLAG_DISABLED or
                                       V4L2_CTRL_FLAG_READ_ONLY or
                                       V4L2_CTRL_FLAG_INACTIVE)) = 0 then begin
    result := Video.GetFeatureValue(Video.UserControls[i].id);
  end else begin
    result := 0;
  end;
end;

procedure TFCamFeatures.FormShow(Sender: TObject);
var i: integer;
begin
  // Fill the string grid with the available features
  for i := 0 to length(Video.UserControls) - 1 do begin
    SGVideoControls.InsertColRow(false, i + 1);
    //SGVideoControls.Objects[0, i + 1] := Tobject(@Video.UserControls[i]);
    SGVideoControls.Cells[0, i + 1] := pchar(@Video.UserControls[i].name[0]) ;
    SGVideoControls.Cells[1, i + 1] := inttostr(SafeGetFeatureValue(i));

    //SGVideoControls.Cells[2, i + 1] := inttostr(Video.UserControls[i].id - V4L2_CID_BASE);
    SGVideoControls.Cells[2, i + 1] := inttostr(Video.UserControls[i].minimum);
    SGVideoControls.Cells[3, i + 1] := inttostr(Video.UserControls[i].maximum);
    SGVideoControls.Cells[4, i + 1] := inttostr(Video.UserControls[i].flags);
  end;

  // Init the current selected feature
  curFeatureIdx := SGVideoControls.Selection.Top - 1;
  if curFeatureIdx < 0 then exit;
  RefreshFeatures();

end;


procedure TFCamFeatures.BFeatureValueSetClick(Sender: TObject);
var newvalue: integer;
begin
  if curFeatureIdx < 0 then exit;

  newvalue := SafeGetFeatureValue(curFeatureIdx);
  newvalue := StrToIntDef(EditFeatureValue.Text, newvalue);
  SetCurFeatureValue(newvalue);
end;

procedure TFCamFeatures.BLoadClick(Sender: TObject);
var i: integer;
    value: dword;
begin
  for i := 0 to length(Video.UserControls) - 1 do begin
    value := IniPropStorage.ReadInteger(inttostr(Video.UserControls[i].id),
                                        SafeGetFeatureValue(i));
    Video.TrySetFeatureValue(Video.UserControls[i].id, value);
  end;
  RefreshFeatures();
end;

procedure TFCamFeatures.BSaveClick(Sender: TObject);
var i: integer;
begin
  for i := 0 to length(Video.UserControls) - 1 do begin
    IniPropStorage.WriteInteger(inttostr(Video.UserControls[i].id),
                                SafeGetFeatureValue(i));
  end;
end;


//V4L2_CTRL_FLAG_DISABLED = $0001;
//V4L2_CTRL_FLAG_GRABBED = $0002;
//V4L2_CTRL_FLAG_READ_ONLY = $0004;
//V4L2_CTRL_FLAG_UPDATE = $0008;
//V4L2_CTRL_FLAG_INACTIVE = $0010;
//V4L2_CTRL_FLAG_SLIDER = $0020;

procedure TFCamFeatures.SetCurFeatureValue(newvalue: dword);
begin
  if curFeatureIdx < 0 then exit;
  // Discard change events because is an internal update
  if updatingHeader > 0 then exit;
  // Beware of feastures that are read only, disabled or inactive
  //if (Video.UserControls[curFeatureIdx].flags and (V4L2_CTRL_FLAG_DISABLED or
  //                                                 V4L2_CTRL_FLAG_READ_ONLY or
  //                                                 V4L2_CTRL_FLAG_INACTIVE)) = 0 then begin
    Video.SetFeatureValue(Video.UserControls[curFeatureIdx].id, newvalue);
  //end;
  RefreshFeatures();
end;


procedure TFCamFeatures.SBFeatureValueChange(Sender: TObject);
begin
  SetCurFeatureValue(SBFeatureValue.Position);
end;

procedure TFCamFeatures.RefreshFeatures;
var i: integer;
    value: dword;
begin
  // Refresh the values column
  for i := 0 to length(Video.UserControls) - 1 do begin
    SGVideoControls.Cells[1, i + 1] := inttostr(SafeGetFeatureValue(i));
  end;

  if curFeatureIdx < 0 then exit;

  // Update the edit header
  LabelFeature.Caption := pchar(@Video.UserControls[curFeatureIdx].name[0]);

  value := SafeGetFeatureValue(curFeatureIdx);
  EditFeatureValue.Text := inttostr(value);

  // Discard change events while doing it
  updatingHeader := 1;
  SBFeatureValue.Min := Video.UserControls[curFeatureIdx].minimum;
  SBFeatureValue.Max := Video.UserControls[curFeatureIdx].maximum;
  SBFeatureValue.Position := value;
  updatingHeader := 0;
end;



procedure TFCamFeatures.FormCreate(Sender: TObject);
begin
  IniPropStorage.IniFileName := FMain.IniPropStorage.IniFileName;
end;

procedure TFCamFeatures.SGVideoControlsSelectCell(Sender: TObject; aCol,
  aRow: Integer; var CanSelect: Boolean);
begin
  curFeatureIdx := aRow - 1;
  if aRow = 0 then exit;
  RefreshFeatures();
end;

initialization
  {$I camfeatures.lrs}

end.

