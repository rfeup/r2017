unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, TAGraph, TASeries, lNetComponents,
  SdpoSerial, SdpoJoystick, Forms, Controls, Graphics, Dialogs,
  StdCtrls, IniPropStorage, ExtCtrls, ComCtrls, Math,
  motor, lNet, RLan, lclintf;

type

  { TFMain }

  TFMain = class(TForm)
    BPIDSet: TButton;
    BSetServos: TButton;
    BWxSet: TButton;
    BVelsSet: TButton;
    BOpenComPort: TButton;
    BCloseComPort: TButton;
    BSerialSendRaw: TButton;
    BReset: TButton;
    BMxSet: TButton;
    BStop: TButton;
    BSetConfig: TButton;
    CBChartCurrentActive: TCheckBox;
    CBExcel: TCheckBox;
    BMemoClear: TButton;
    CBRawDebug: TCheckBox;
    CBContinous: TCheckBox;
    CBPID: TCheckBox;
    CBJoystick: TCheckBox;
    CBRemoteSpeeds: TCheckBox;
    EditJoyX: TEdit;
    EditJoyDevice: TEdit;
    EditJoyB: TEdit;
    EditKf: TEdit;
    EditKp: TEdit;
    EditKi: TEdit;
    EditKd: TEdit;
    EditServoRightLock: TEdit;
    EditServoLeftLow: TEdit;
    EditServoRightLow: TEdit;
    EditServoLeftUp: TEdit;
    EditServoLeftLock: TEdit;
    EditServoRightUp: TEdit;
    EditW1: TEdit;
    EditW2: TEdit;
    EditW3: TEdit;
    EditTicksToRads: TEdit;
    EditWheelDiameter: TEdit;
    EditV: TEdit;
    EditVn: TEdit;
    EditW: TEdit;
    EditWheelDist: TEdit;
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
    Label20: TLabel;
    Label21: TLabel;
    Label22: TLabel;
    Label23: TLabel;
    Label24: TLabel;
    Label5: TLabel;
    Label6: TLabel;
    Label7: TLabel;
    Label8: TLabel;
    Label9: TLabel;
    TabNet: TTabSheet;
    UDPVel: TLUDPComponent;
    RGServosPosition: TRadioGroup;
    SBW1: TScrollBar;
    SBW2: TScrollBar;
    SBW3: TScrollBar;
    SBV: TScrollBar;
    SBVn: TScrollBar;
    SBW: TScrollBar;
    Joystick: TSdpoJoystick;
    SeriesSpeedM2: TLineSeries;
    SeriesSpeedM3: TLineSeries;
    SeriesCurrentM2: TLineSeries;
    SeriesCurrentM3: TLineSeries;
    ChartSpeeds: TChart;
    CBSyncPWM: TCheckBox;
    CBXY: TCheckBox;
    ChartCurrents: TChart;
    CBChartSpeedActive: TCheckBox;
    EditM1: TEdit;
    EditM1Delta: TEdit;
    EditM1Decimate: TEdit;
    EditM2: TEdit;
    EditM3: TEdit;
    Label2: TLabel;
    Label3: TLabel;
    Label4: TLabel;
    PageControl: TPageControl;
    SBM1: TScrollBar;
    SBM2: TScrollBar;
    SBM3: TScrollBar;
    SeriesCurrentM1: TLineSeries;
    SeriesSpeedM1: TLineSeries;
    EditRawData: TEdit;
    MemoDebug: TMemo;
    Serial: TSdpoSerial;
    EditComPort: TEdit;
    IniPropStorage: TIniPropStorage;
    Label1: TLabel;
    TabCurrent: TTabSheet;
    TabDebug: TTabSheet;
    TabConfig: TTabSheet;
    TabServos: TTabSheet;
    TabJoystick: TTabSheet;
    TabSpeed: TTabSheet;
    procedure BCloseComPortClick(Sender: TObject);
    procedure BMemoClearClick(Sender: TObject);
    procedure BMxSetClick(Sender: TObject);
    procedure BOpenComPortClick(Sender: TObject);
    procedure BPIDSetClick(Sender: TObject);
    procedure BResetClick(Sender: TObject);
    procedure BSerialSendRawClick(Sender: TObject);
    procedure BSetConfigClick(Sender: TObject);
    procedure BSetServosClick(Sender: TObject);
    procedure BStopClick(Sender: TObject);
    procedure BVelsSetClick(Sender: TObject);
    procedure BWxSetClick(Sender: TObject);
    procedure CBJoystickClick(Sender: TObject);
    procedure CBXYChange(Sender: TObject);
    procedure EditMxKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure FormShow(Sender: TObject);
    procedure UDPVelError(const msg: string; aSocket: TLSocket);
    procedure UDPVelReceive(aSocket: TLSocket);
    procedure RGServosPositionClick(Sender: TObject);
    procedure SBMxChange(Sender: TObject);
    procedure SBVVnWChange(Sender: TObject);
    procedure SBWxChange(Sender: TObject);
    procedure SerialRxData(Sender: TObject);
    procedure EditVelsKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState
      );
    procedure EditWxKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState);
  private
    procedure processFrame(channel: char; value: integer);
    procedure ReceiveData(s: string);
    procedure SendChannel(channel: char; value: integer);
    procedure SendMotorPWM(MotorChannel: char; PWM: integer);
    procedure SendOdo;
    procedure SetComState(newState: boolean);
    procedure SetServosPosition;
    { private declarations }
  public
    serialData: string;

    channel: char;
    nfile,frame: integer;
    frameData: string;

    dt: double;
    TicsToRads, WheelDiameter, WheelDist: double;

    Set_PWM_M: array [1..3] of integer;
    PWM_M: array [1..3] of integer;
    Current:  array [1..3] of integer;
    Odo: array [1..3] of Smallint;
    Wr: array [1..3] of double;
    RefWr: array [1..3] of double;

    actOdoM: array [1..3] of integer;
    PID: array [1..3] of TPID;
    V, Vn, W: double;
    maxWr: double;
    Vbat: double;

    ServoLeftLow, ServoLeftUp , ServoLeftLock: integer;
    ServoRightLow, ServoRightUp , ServoRightLock: integer;
    ServoLeftPosition, ServoRightPosition: integer;

    RemoteV, RemoteVn, RemoteW: double;
    RemoteTime: qword;

    sample: integer;
    M1Decimate, M1DecimationCount: integer;
    sline: string;
    procedure Debug(s: string);
  end;

var
  FMain: TFMain;
  NetInBuf: TUDPBuffer;
  NetOutBuf: TUDPBuffer;
  Servo_pos:Integer;


procedure SendMessage(c: char; val: integer);
procedure SendRaw(s: string);


implementation


{$R *.lfm}

procedure SendMessage(c: char; val: integer);
begin
  FMain.Serial.WriteData(c + IntToHex(word(Val) and $FFFF, 4));
end;

procedure SendRaw(s: string);
begin
  FMain.Serial.WriteData(s);
end;



{ TFMain }

procedure TFMain.SetComState(newState: boolean);
var comColor: TColor;
begin
  try
  if newState then begin
    Serial.Device := EditComPort.Text;
    Serial.Open;
  end else begin
    Serial.Close;
  end;
  finally
    if Serial.Active then comColor := clGreen
    else comColor := clRed;
    EditComPort.Color := comColor;
  end;
end;



procedure TFMain.Debug(s: string);
begin
  MemoDebug.Lines.Add(s);
  while MemoDebug.Lines.Count > 1000 do begin
    MemoDebug.Lines.Delete(0);
  end;
end;


procedure TFMain.SendChannel(channel: char; value: integer);
begin
  if Serial.Active then begin
    Serial.WriteData(channel + IntToHex(value, 4));
  end;
end;

procedure TFMain.BOpenComPortClick(Sender: TObject);
begin
  SetComState(true);
end;

procedure TFMain.BPIDSetClick(Sender: TObject);
var i: integer;
begin
  for i := 1 to 3 do begin
    with PID[i] do begin
      Kf := strtofloat(EditKf.Text);
      Kp := strtofloat(EditKp.Text);
      Ki := strtofloat(EditKi.Text);
      Kd := strtofloat(EditKd.Text);
      ControlPeriod := dt;
      m_sat := 10; //TODO Config max voltage
      ek := 0;
      ek_1 := 0;
      Sek := 0;
    end;
  end;
end;


procedure TFMain.BResetClick(Sender: TObject);
begin
  if Serial.Active then begin
    Serial.SetDTR(false);
    Serial.SetDTR(true);
  end;
end;

procedure TFMain.BSerialSendRawClick(Sender: TObject);
begin
  Serial.WriteData(EditRawData.Text);
end;

procedure TFMain.BMemoClearClick(Sender: TObject);
begin
  MemoDebug.Clear;
  sample := 0;
  M1Decimate := StrToIntDef(EditM1Decimate.Text, 0);
  M1DecimationCount := 0;
end;

procedure TFMain.BSetConfigClick(Sender: TObject);
begin
  TicsToRads := strtofloat(EditTicksToRads.Text);
  WheelDiameter := strtofloat(EditWheelDiameter.Text);
  WheelDist := strtofloat(EditWheelDist.Text);
end;

procedure TFMain.SetServosPosition;
begin
  //case RGServosPosition.ItemIndex of
  case Servo_pos of
    0: begin
      ServoLeftPosition := ServoLeftLow;
      ServoRightPosition:= ServoRightLow;
    end;
    1: begin
      ServoLeftPosition := ServoLeftUp;
      ServoRightPosition:= ServoRightUP;
    end;
    2: begin
      ServoLeftPosition := ServoLeftLock;
      ServoRightPosition:= ServoRightLock;
    end;
  end;

end;


procedure TFMain.BSetServosClick(Sender: TObject);
begin
  ServoLeftLow := StrToInt(EditServoLeftLow.Text);
  ServoLeftUp := StrToInt(EditServoLeftUp.Text);
  ServoLeftLock := StrToInt(EditServoLeftLock.Text);

  ServoRightLow := StrToInt(EditServoRightLow.Text);
  ServoRightUp := StrToInt(EditServoRightUp.Text);
  ServoRightLock := StrToInt(EditServoRightLock.Text);
  Servo_pos:=RGServosPosition.ItemIndex;
  SetServosPosition();
end;

procedure TFMain.BStopClick(Sender: TObject);
begin
  SBM1.Position := 0;
  SBM2.Position := 0;
  SBM3.Position := 0;

  SBW1.Position := 0;
  SBW2.Position := 0;
  SBW3.Position := 0;

  SBV.Position := 0;
  SBVn.Position := 0;
  SBW.Position := 0;
end;

procedure TFMain.BMxSetClick(Sender: TObject);
begin
  SBM1.Position := StrToIntDef(EditM1.Text, 0);
  SBM2.Position := StrToIntDef(EditM2.Text, 0);
  SBM3.Position := StrToIntDef(EditM3.Text, 0);
end;


procedure TFMain.BVelsSetClick(Sender: TObject);
begin
  SBV.Position := round(1000 * StrToFloatDef(EditV.Text, 0));
  SBVn.Position := round(1000 * StrToFloatDef(EditVn.Text, 0));
  SBW.Position := round(1000 * StrToFloatDef(EditW.Text, 0));
end;

procedure TFMain.BWxSetClick(Sender: TObject);
begin
  SBW1.Position := round(1000 * StrToFloatDef(EditW1.Text, 0) / maxWr);
  SBW2.Position := round(1000 * StrToFloatDef(EditW2.Text, 0) / maxWr);
  SBW3.Position := round(1000 * StrToFloatDef(EditW3.Text, 0) / maxWr);
end;

procedure TFMain.CBJoystickClick(Sender: TObject);
begin
  if CBJoystick.Checked then begin
    Joystick.DeviceLin := EditJoyDevice.Text;
    Joystick.Active := true;
  end else begin
    Joystick.Active := false;
  end;
end;

procedure TFMain.CBXYChange(Sender: TObject);
begin
  SeriesCurrentM1.Clear;
  SeriesSpeedM1.Clear
end;



procedure TFMain.EditMxKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = 13 then BMxSet.Click;
end;

procedure TFMain.EditVelsKeyDown(Sender: TObject; var Key: Word; Shift: TShiftState
  );
begin
  if key = 13 then BVelsSet.Click;
end;

procedure TFMain.EditWxKeyDown(Sender: TObject; var Key: Word;
  Shift: TShiftState);
begin
  if key = 13 then BWxSet.Click;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  IniPropStorage.WriteBoolean('COM_port_state', Serial.Active);
  Serial.Active := false;
end;

procedure TFMain.FormCreate(Sender: TObject);
begin
  M1DecimationCount := 0;
  sample := 0;
  dt := 0.05;
  maxWr := 40;
  Vbat := 12;
  UDPVel.Connect('127.0.0.1',9006);
  UDPVel.Listen(9006)
  //EditM1.tag := ptrint(SBM1);
end;

procedure TFMain.FormShow(Sender: TObject);
var openSerial: boolean;
begin
  BSetConfig.Click;
  BPIDSet.Click;
  BSetServos.Click;
  openSerial := IniPropStorage.ReadBoolean('COM_port_state', true);
  SetComState(openSerial);
  BMemoClear.Click;
end;

procedure TFMain.UDPVelError(const msg: string; aSocket: TLSocket);
begin

end;

procedure TFMain.UDPVelReceive(aSocket: TLSocket);
var ii,NumberBytes: integer;
    dd: word;
    Messa: String;
begin
    UDPVel.GetMessage(Messa);
    if messa = '' then begin
      RemoteV := 0;
      RemoteVN:= 0;
      RemoteW := 0;
      Servo_pos:= 0;
      exit;
    end;
    RemoteTime := GetTickCount64();

    ClearUDPBuffer(NetInBuf);
    NumberBytes := length(Messa);
    if NumberBytes >= UDPBufSize then
      exit;
    NetInBuf.MessSize := NumberBytes;
    NetInBuf.ReadDisp := 0;
    move(Messa[1], NetInBuf.data[0], NumberBytes);
    if chr(NetGetByte(NetInBuf)) <> 'V' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'N' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'W' then
       exit;
    if chr(NetGetByte(NetInBuf)) <> 'S' then
       exit;

    RemoteV :=netgetint(NetInBuf)/1000;
    RemoteVN:=netgetint(NetInBuf)/1000;
    RemoteW :=netgetint(NetInBuf)/1000;
    MemoDebug.lines.add(floattostr(RemoteV)) ;
     MemoDebug.lines.add(floattostr(RemoteVn)) ;
      MemoDebug.lines.add(floattostr(RemoteW)) ;
    Servo_pos := round(netgetint(NetInBuf)/1000);
    SetServosPosition;
   // UDPVel.SendMessage('V:'+#10+FloatToStr(RemoteV)+#10+'Vn:'+#10+FloatToStr(RemoteVn)+#10+'W:'+#10+FloatToStr(RemoteW)+#10,'127.0.0.1:9808');
end;



procedure TFMain.RGServosPositionClick(Sender: TObject);
begin
  SetServosPosition();
end;



procedure TFMain.SBMxChange(Sender: TObject);
var SBMx: TScrollBar;
    EditMx: TEdit;
begin
  SBMx := Sender as TScrollBar;
  Set_PWM_M[1 + SBMx.tag] := SBMx.Position;

  if CBSyncPWM.Checked then begin
    SendMotorPWM(char(ord('R') + SBMx.tag), Set_PWM_M[1 + SBMx.tag]);
  end else begin
    SendMotorPWM(char(ord('M') + SBMx.tag), Set_PWM_M[1 + SBMx.tag]);
  end;
  EditMx := TEdit(FindChildControl('EditM' + inttostr(SBMx.tag + 1)));
  if Assigned(EditMx) then
    EditMx.Text := IntToStr(Set_PWM_M[1 + SBMx.tag]);
end;



procedure TFMain.SBVVnWChange(Sender: TObject);
var i: integer;
begin
  V := SBV.Position / 1000;
  EditV.Text := format('%g', [V]);
  Vn := SBVn.Position / 1000;
  EditVn.Text := format('%g', [Vn]);
  W := SBW.Position / 1000;
  Editw.Text := format('%g', [W]);

  RefWr[1] := -(-sqrt(3)/2 * V + 0.5 * Vn + WheelDist * W) / (0.5 * WheelDiameter);
  RefWr[2] := -(                      -Vn + WheelDist * W) / (0.5 * WheelDiameter);
  RefWr[3] := -( sqrt(3)/2 * V + 0.5 * Vn + WheelDist * W) / (0.5 * WheelDiameter);

  SBW1.Position := round(RefWr[1] * 1000 / maxWr);
  SBW2.Position := round(RefWr[2] * 1000 / maxWr);
  SBW3.Position := round(RefWr[3] * 1000 / maxWr);
end;



procedure TFMain.SBWxChange(Sender: TObject);
var SBWx: TScrollBar;
    EditWx: TEdit;
begin
  SBWx := Sender as TScrollBar;
  RefWr[1 + SBWx.tag] := maxWr * SBWx.Position / 1000;

  EditWx := TEdit(FindChildControl('EditW' + inttostr(SBWx.tag + 1)));
  if Assigned(EditWx) then
    EditWx.Text := format('%.3g',[RefWr[1 + SBWx.tag]]) ;
end;

// PWM between -1024 and 1024
procedure TFMain.SendMotorPWM(MotorChannel: char; PWM: integer);
begin
  if PWM >= 0 then begin
    Serial.WriteData(MotorChannel + IntToHex(PWM, 4));
  end else begin
    Serial.WriteData(MotorChannel + '1' + IntToHex(abs(PWM), 3));
  end;
end;

function isHexDigit(c: char): boolean;
begin
  result := c in ['0'..'9', 'A'..'F'];
end;

procedure TFMain.SendOdo();
var ld:word;
begin

    ClearUDPBuffer(NetOutBuf);
     //
    NetPutByte(NetOutBuf, ord('V'));
    NetPutByte(NetOutBuf, ord('W'));
    NetPutByte(NetOutBuf, ord('3'));

    ld := round(Wr[3]*1000);
    NetPutShort(NetOutBuf, ld);

    ld := round(Wr[1]*1000);
    NetPutShort(NetOutBuf, ld);

    ld := round(Wr[2]*1000);
    NetPutShort(NetOutBuf, ld);

    UDPVel.Send(NetOutBuf.data, NetOutBuf.MessSize, '127.0.0.1' + ':9001');
end;

procedure TFMain.processFrame(channel: char; value: integer);
var s: string;
    i: integer;
    SeriesCurrentMx, SeriesSpeedMx: TLineSeries;
    actTime: QWord;
begin
  //MemoDebug.Text := MemoDebug.Text + channel;
  if channel = 'p' then begin // Metapacket delimiter
    if value = 0 then begin   // End of metapacket

      if CBJoystick.Checked then begin
        EditJoyX.text := format('%d, %d, %d', [Joystick.Axis[0], Joystick.Axis[1], Joystick.Axis[2]]);
        V := -0.5 * (Joystick.Axis[5] / 32768); //1
        Vn := -0.5 * (Joystick.Axis[2] / 32768);   //0
        W := -5 * (Joystick.Axis[0] / 32768);    //2

        SBV.Position := round(1000 * V);
        SBVn.Position := round(1000 * Vn);
        SBW.Position := round(1000 * W);

        EditJoyB.text := format('%d, %d, %d', [Joystick.Buttons[0], Joystick.Buttons[1], Joystick.Buttons[2]]);
        if Joystick.Buttons[0] > 0 then
          RGServosPosition.ItemIndex := 0;
        if Joystick.Buttons[1] > 0 then
          RGServosPosition.ItemIndex := 1;
      end;

      if CBRemoteSpeeds.Checked then begin
        actTime := GetTickCount64();

        if actTime - RemoteTime > 300 then begin
          RemoteV := 0;
          RemoteVN:= 0;
          RemoteW := 0;
          Servo_pos:= 0;
        end;

        SBV.Position := round(1000 * RemoteV);
        SBVn.Position := round(1000 * RemoteVn);
        SBW.Position := round(1000 * RemoteW);

        RGServosPosition.ItemIndex := Servo_pos;

      end;

      if CBContinous.Checked then begin
        if CBPID.Checked then begin
          for i := 1 to 3 do begin
            Set_PWM_M[i] := round(1024 * CalcPID(PID[i], RefWr[i], Wr[i]) / Vbat);
          end;
          SBM1.Position := Set_PWM_M[1];
          SBM2.Position := Set_PWM_M[2];
          SBM3.Position := Set_PWM_M[3];
        end else begin
          Set_PWM_M[1] := SBM1.Position;
          Set_PWM_M[2] := SBM2.Position;
          Set_PWM_M[3] := SBM3.Position;
        end;

        for i := 1 to 3 do begin
          if CBSyncPWM.Checked then begin
            SendMotorPWM(char(ord('R') + i - 1), Set_PWM_M[i]);
          end else begin
            SendMotorPWM(char(ord('M') + i - 1), Set_PWM_M[i]);
          end;
        end;
        Serial.WriteData('G' + IntToHex(ServoLeftPosition, 4));
        Serial.WriteData('H' + IntToHex(ServoRightPosition, 4));
      end;

      sline := '';
      for i := 1 to 3 do begin

        // Show Current
        if CBChartCurrentActive.Checked then begin
          SeriesCurrentMx := TLineSeries(ChartCurrents.Series[i - 1]);
          if not CBXY.Checked then begin
            SeriesCurrentMx.Add(5/1025 * Current[i] / 1e-3);
          end;
          while SeriesCurrentMx.Count > 10 * 50 do begin
            SeriesCurrentMx.Delete(0);
          end;
        end;

        // Show Speed
        if CBChartSpeedActive.Checked then begin
          SeriesSpeedMx := TLineSeries(ChartSpeeds.Series[i - 1]);
          if CBXY.Checked then begin
            SeriesSpeedMx.AddXY(PWM_M[i], Wr[i]);
          end else begin
            SeriesSpeedMx.Add(Wr[i]);
          end;
          while SeriesSpeedMx.Count > 10 * 50 do begin
            SeriesSpeedMx.Delete(0);
          end;
        end;

        sline := format('%d %d', [sample, PWM_M[i]]);
        sline := sline +  format(' %d %d', [Current[i], Odo[i]]);
        if CBExcel.Checked then begin
          Debug(sline);
        end;

      end;

      if M1DecimationCount >= M1Decimate then begin
        SBM1.Position := SBM1.Position + StrToIntDef(EditM1Delta.Text, 0);
        M1DecimationCount := 0;
      end;
      inc(M1DecimationCount);

      inc(sample);
    end;
  end else if channel in ['m', 'n', 'o'] then begin
    i := 1 + ord(channel) - ord('m');
    PWM_M[i] := value;
  end else if channel in ['i', 'j', 'k'] then begin
    i := 1 + ord(channel) - ord('i');
    Current[i] := value;

  end else if channel in ['r', 's', 't'] then begin
    i := 1 + ord(channel) - ord('r');
    if sample <= 2 then actOdoM[i] := value; // First sample must be discarded

    Odo[i] := Smallint(value) - Smallint(actOdoM[i]);
    Wr[i] := Odo[i] * TicsToRads / dt;
    actOdoM[i] := value;
    if channel = 't' then SendOdo();

  end;
end;


// Sample Pwm I speed

procedure TFMain.ReceiveData(s: string);
var //b: byte;
    c: char;
    value: integer;
begin
  if s = '' then exit;
  serialData := serialData + s;

  if CBRawDebug.Checked then begin
    Debug(s);
  end;

  while Length(serialData) > 0 do begin
    c := serialData[1];
    serialData := copy(serialData, 2, maxint);
    if frame = -1 then begin
      if (c in ['G'..']']) or (c in ['g'..'}']) or (c = ';') then begin
        frame := 0;
        channel := c;
        frameData := '';
      end;
    end else begin
      if isHexDigit(c) then begin
        frameData := frameData + c;
        inc(frame);
        if frame = 4 then begin
          value := StrToIntDef('$' + frameData, -1);
          processFrame(channel, value);
          frame := -1;
        end;
      end else begin
        frame := -1;
      end;
    end;
  end;
end;


procedure TFMain.SerialRxData(Sender: TObject);
var s: string;
begin
  s := Serial.ReadData;
  ReceiveData(s);
end;

procedure TFMain.BCloseComPortClick(Sender: TObject);
begin
  SetComState(false);
end;


end.
