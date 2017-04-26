unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, LResources, Forms, Controls, Graphics, Dialogs, StdCtrls,
  SdpoVideo4L2, VideoDev2, SdpoFastForm, LCLIntf, ComCtrls, ValEdit, Grids,
  rlan, IniPropStorage, TAGraph, TASeries, lNetComponents, CamFeatures, lNet, math;

type
  PRGB24Pixel = ^TRGB24Pixel;
  TRGB24Pixel = packed record
    Red:   Byte;
    Green: Byte;
    Blue:  Byte;
  end;

  PBGR24Pixel = ^TBGR24Pixel;
  TBGR24Pixel = packed record
    Blue:  Byte;
    Green: Byte;
    Red:   Byte;
  end;

  PTRGBAcc = ^TRGBAcc;
  TRGBAcc = record
    R, G, B, M: integer;
    col: integer;
    active: boolean;
  end;

  TRGBAccBand = record
    acc: array of TRGBAcc;
    count: integer;
  end;

  { TFMain }

  TFMain = class(TForm)
    CBVideoActive: TCheckBox;
    Chart1: TChart;
    CBChartActive: TCheckBox;
    EditDebug: TEdit;
    EditLine: TEdit;
    EditLineCount: TEdit;
    EditMinTresh: TEdit;
    Label1: TLabel;
    LineSeriesRED: TLineSeries;
    LineSeriesBlue: TLineSeries;
    LineSeriesGreen: TLineSeries;
    EditDevice: TEdit;
    Image: TSdpoFastForm;
    IniPropStorage: TIniPropStorage;
    UDPRob: TLUDPComponent;
    Memo: TMemo;
    StatusBar: TStatusBar;
    Video: TSdpoVideo4L2;
    procedure FormDestroy(Sender: TObject);
    procedure SendCameraMessage(mtype: integer);
    procedure CBVideoActiveChange(Sender: TObject);
    procedure FormClose(Sender: TObject; var CloseAction: TCloseAction);
    procedure FormCreate(Sender: TObject);
    procedure VideoFrame(Sender: TObject; FramePtr: PByte);
    //procedure qsort(var a : TMaxColorArray);
  private
    { private declarations }
  public
    { public declarations }
    FrameRate: integer;
    FrameTime : DWord;

    RGBSort, RGBPeaks: TList;
  end;

var
  FMain: TFMain;
  NetOutBuf: TUDPBuffer;

//const
//  MIN_Color = 90;

implementation

{ TFMain }

procedure TFMain.SendCameraMessage(mtype: integer);
var i: integer;
    ld: string;
begin
    ClearUDPBuffer(NetOutBuf);
    NetPutWord(NetOutBuf, RGBPeaks.Count);
    for i := 0 to RGBPeaks.Count - 1 do begin
      NetPutWord(NetOutBuf, PTRGBAcc(RGBPeaks.Items[i])^.col);
      NetPutWord(NetOutBuf, PTRGBAcc(RGBPeaks.Items[i])^.R);
      NetPutWord(NetOutBuf, PTRGBAcc(RGBPeaks.Items[i])^.G);
      NetPutWord(NetOutBuf, PTRGBAcc(RGBPeaks.Items[i])^.B);
    end;

{    NetPutByte(NetOutBuf, ord('1'));
    NetPutByte(NetOutBuf, ord('2'));
    NetPutByte(NetOutBuf, ord('3'));
    NetPutByte(NetOutBuf, ord('4'));
    NetPutByte(NetOutBuf, ord('5'));

    ld := max_index_array[0].Color;
    NetPutString(NetOutBuf, ld);

    ld := max_index_array[1].Color;
    NetPutString(NetOutBuf, ld);

    ld := max_index_array[2].Color;
    NetPutString(NetOutBuf, ld);

    ld := max_index_array[3].Color;
    NetPutString(NetOutBuf, ld);

    ld := max_index_array[4].Color;
    NetPutString(NetOutBuf, ld);
}
    UDPRob.Send(NetOutBuf.data, NetOutBuf.MessSize, '127.0.0.1:9020');
end;

procedure TFMain.FormDestroy(Sender: TObject);
begin
  RGBSort.Free;
  RGBPeaks.Free;
  UDPRob.Disconnect();
end;

{procedure TFMain.qsort(var a : TMaxColorArray);

    procedure sort(l,r: longint);
      var
         i,j,x: longint;
         y : TMaxColor;
      begin
         i:=l;
         j:=r;
         x:=a[(l+r) div 2].Index;
         repeat
           while a[i].Index < x do
            inc(i);
           while x<a[j].Index do
            dec(j);
           if not(i>j) then
             begin
                y:=a[i];
                a[i]:=a[j];
                a[j]:=y;
                inc(i);
                j:=j-1;
             end;
         until i>j;
         if l<j then
           sort(l,j);
         if i<r then
           sort(i,r);
      end;

begin
    sort(0,Length(a)-1);
end;
}

procedure TFMain.CBVideoActiveChange(Sender: TObject);
begin
  if CBVideoActive.Checked then begin
    Video.Device:=EditDevice.Text;
    if not (Video.PixelFormat in [uvcpf_YUYV, uvcpf_YUV420, uvcpf_RGB24, uvcpf_BGR24]) then
    begin
      ShowMessage('This little demo program can only display YUYV, '
                + 'YUV420, RGB24 or BGR24. If you are using libv4l '
                + 'then it should be possible to request at least '
                + 'RGB24, BGR24 or YUV420 from all supported v4l2 '
                + 'devices.');
    end;
    Image.Width := Video.Width;
    Image.Height := Video.Height;
    Image.Show;

    Video.SetDebugList(Memo.Lines);
    Video.Open;

    Video.GetUserControls;

    FCamFeatures.Video := Video;
    FCamFeatures.show;
    //Video.SetFeatureValue(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO);

    Video.SetFeatureValue(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    Video.SetFeatureValue(V4L2_CID_AUTOGAIN, 0);
    //Video.SetFeatureValue(V4L2_CID_AUTO_WHITE_BALANCE, 0);

  end else begin
    Video.Close;
    Image.Close;
  end;
end;

procedure TFMain.FormClose(Sender: TObject; var CloseAction: TCloseAction);
begin
  FCamFeatures.Close;
  Video.Close;
  Image.Close;
end;

procedure TFMain.FormCreate(Sender: TObject);
var path: string;
begin
  DefaultFormatSettings.DecimalSeparator := '.';

  RGBSort := TList.Create;
  RGBPeaks := TList.Create;

  path := ExtractFilePath(Application.ExeName) + DirectorySeparator;
  if Paramcount > 0 then begin
    IniPropStorage.IniFileName := path + ParamStr(1);
  end else begin
    IniPropStorage.IniFileName := path + 'config.ini';
  end;

  UDPRob.Connect('127.0.0.1', 9021);
end;

// this format (like many others) is not guaranteed
// to be supported by all devices.
procedure YUYV_to_Gray(src: PLongWord; dest: PLongWord; size: integer);
var i, r,g,b: integer;
begin
  for i := 0 to (size div 2) - 1 do begin
    g := src^ and $FF;
    r := g;
    b := g;
    dest^ := (r shl 16) or (g shl 8) or (b);
    Inc(dest);
    g := (src^ shr 16) and $FF;
    r := g;
    b := g;
    dest^ := (r shl 16) or (g shl 8) or (b);
    Inc(src);
    Inc(dest);
  end;
end;

// when using the libv4l wrapper then YUV420 is always
// amongst the supported formats, even if the camera
// would not support it natively.
procedure YUV420_to_Gray(src: PByte; dest: PLongWord; size: integer);
var i, r,g,b: integer;
begin
  // Y, U and V are not interleaved, they are in continuous
  // blocks. We read the first block containing Y (8 bit per pixel)
  // and simply ignore the rest.
  for i := 0 to size -1 do begin
    g := src[i];
    r := g;
    b := g;
    dest[i] := (r shl 16) or (g shl 8) or (b);
  end;
end;

// when using the libv4l wrapper then RGB24 is always
// amongst the supported formats, even if the camera
// would not support it natively.
procedure RGB24_to_TrueColor(src: PRGB24Pixel; dest: PLongWord; size: Integer);
var i: integer;
begin
  for i := 0 to size -1 do begin
    dest[i] :=  (src[i].Red shl 16) or (src[i].Green shl 8) or src[i].Blue;
  end;
end;

// when using the libv4l wrapper then BGR24 is always
// amongst the supported formats, even if the camera
// would not support it natively.
procedure BGR24_to_TrueColor(src: PRGB24Pixel; dest: PLongWord; size: Integer);
var i: integer;
begin
  //move(src^, dest^, size*4);
  for i := 0 to size -1 do begin
    // this is the reason why often BGR instead of RGB
    // is used, we don't need shifting like above, we can
    // simply dump entire longwords into their new locations.
    dest[i] := PLongWord(@src[i])^;
  end;
end;

function SortRGBAcc(Item1, Item2: Pointer): Integer;
var I1, I2: PTRGBAcc;
begin
  I1 := PTRGBAcc(Item1);
  I2 := PTRGBAcc(Item2);
  if I1^.M > I2^.M then begin
    result := -1;
  end else if I1^.M < I2^.M then begin
    result := 1;
  end else begin
    result := 0;
  end;
end;

function SortRGBPeaks(Item1, Item2: Pointer): Integer;
var I1, I2: PTRGBAcc;
begin
  I1 := PTRGBAcc(Item1);
  I2 := PTRGBAcc(Item2);
  if I1^.col > I2^.col then begin
    result := 1;
  end else if I1^.col < I2^.col then begin
    result := -1;
  end else begin
    result := 0;
  end;
end;


procedure TFMain.VideoFrame(Sender: TObject; FramePtr: PByte);
var lineCount, line, i, j, k: integer;
    BGR24Pixel: TBGR24Pixel;
    AccBand: TRGBAccBand;
    minthresh, c: integer;
    s: string;
begin
  //FrameRate:=round(1/((GetTickCount-FrameTime)/1000)*0.5 + FrameRate*0.5);
  FrameRate:=round(1/((GetTickCount-FrameTime)/1000));
  FrameTime:=GetTickCount;
  StatusBar.SimpleText := format('(%d, %d) %d fps', [video.Width, Video.Height, FrameRate]);

  if Video.Width * Video.Height <> Image.Width * Image.Height then
  begin
    // I have seen this happening when the driver accepted
    // the video dimensions without complaining but then
    // silently set them to its own liking. For the sake of
    // simplicity we simply raise an exception here and don't
    // attempt to fix it.
    raise Exception.Create('Video picture size is different from what we had requested');
  end;

  if CBChartActive.Checked then begin
    LineSeriesBlue.Clear;
    LineSeriesRED.Clear;
    LineSeriesGreen.Clear;
  end;

  line := StrToIntDef(EditLine.Text, 100);
  lineCount := StrToIntDef(EditLineCount.Text, 8);
  AccBand.count:= lineCount;
  SetLength(AccBand.acc, Video.Width);

  // Accumulate the RGB components over the selected lines
  for i := 0 to Video.Width - 1 do begin
    with AccBand.acc[i] do begin
      r := 0;
      g := 0;
      b := 0;

      for j := 0 to lineCount - 1 do begin
        BGR24Pixel := pBGR24Pixel(FramePtr + ((line + j) * Video.Width  + i) * sizeof(BGR24Pixel))^;
        r := r + BGR24Pixel.Red;
        g := g + BGR24Pixel.Green;
        b := b + BGR24Pixel.Blue;
      end;

      if lineCount > 0 then begin
        R := (2 * R) div lineCount;
        G := (2 * G) div lineCount;
        B := (2 * B) div lineCount;
      end;

      M := max(R, max(G, B));
      col := i;
      active := true;

      if CBChartActive.Checked then begin
        // Show in the chart
        LineSeriesRED.AddXY(i, r);
        LineSeriesGreen.AddXY(i, g);
        LineSeriesBlue.AddXY(i, b);
      end;

    end;
  end;

  RGBSort.Clear;
  for i := 0 to  Video.Width - 1 do begin
    RGBSort.Add(@(AccBand.acc[i]));
  end;

  RGBSort.Sort(@SortRGBAcc);

  Memo.Clear;
  {
  Memo.Lines.Add(inttostr(RGBSort.Count));
  for i := 0 to min(10, RGBSort.Count) - 1 do begin
    s := format('%d (%d, %d, %d)', [PTRGBAcc(RGBSort.Items[i])^.col, PTRGBAcc(RGBSort.Items[i])^.R, PTRGBAcc(RGBSort.Items[i])^.G, PTRGBAcc(RGBSort.Items[i])^.B]);
    Memo.Lines.Add(s);
  end;
  }

  minthresh := StrToIntDef(EditMinTResh.text, 8);

  RGBPeaks.Clear;
  for i := 0 to  Video.Width - 1 do begin
    if RGBPeaks.Count >= 5 then break;
    if not PTRGBAcc(RGBSort.Items[i])^.active then continue;
    if PTRGBAcc(RGBSort.Items[i])^.M < minthresh then break;

    RGBPeaks.Add(RGBSort.Items[i]);
    for j := -5 to 5 do begin
      c := PTRGBAcc(RGBSort.Items[i])^.col;
      if (c + j < 0) or (c + j > Video.Width - 1) or (j = 0) then continue;
      AccBand.acc[c + j].active := false;
    end;
  end;

  RGBPeaks.Sort(@SortRGBPeaks);

  Memo.Clear;

  Memo.Lines.Add(inttostr(RGBPeaks.Count));
  for i := 0 to RGBPeaks.Count - 1 do begin
    s := format('%d (%d, %d, %d)', [PTRGBAcc(RGBPeaks.Items[i])^.col, PTRGBAcc(RGBPeaks.Items[i])^.R, PTRGBAcc(RGBPeaks.Items[i])^.G, PTRGBAcc(RGBPeaks.Items[i])^.B]);
    Memo.Lines.Add(s);
  end;

  // Show sampled lines
  for j := 0 to lineCount do begin
    BGR24Pixel.Blue := 255;
    BGR24Pixel.Green := 255;
    BGR24Pixel.Red := 0;
    pBGR24Pixel(FramePtr + ((line + j) * Video.Width + 1) * sizeof(BGR24Pixel))^ := BGR24Pixel;
  end;

  SendCameraMessage(0);

  case Video.PixelFormat of
    uvcpf_YUYV:
      YUYV_to_Gray(PLongWord(FramePtr), PLongWord(Image.Data), Video.Width * Video.Height);
    uvcpf_YUV420:
      YUV420_to_Gray(FramePtr, Image.Data, Video.Width * Video.Height);
    uvcpf_RGB24:
      RGB24_to_TrueColor(PRGB24Pixel(FramePtr), Image.Data, Video.Width * Video.Height);
    uvcpf_BGR24:
      BGR24_to_TrueColor(PRGB24Pixel(FramePtr), Image.Data, Video.Width * Video.Height);
  end;
  Image.Paint;
end;

initialization
  {$I main.lrs}

end.

