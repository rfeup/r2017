unit main;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, LResources, Forms, Controls, Graphics, Dialogs, StdCtrls,
  SdpoVideo4L2, VideoDev2, SdpoFastForm, LCLIntf, ComCtrls, ValEdit, Grids, rlan,
  IniPropStorage, TAGraph, TASeries, lNetComponents, CamFeatures, lNet;

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

  TMaxColor = record
    Index, Max: Integer;
    Color: String;
  end;

  TMaxColorArray = array of TMaxColor;

  TIntArray = array of Integer;
  { TFMain }

  TFMain = class(TForm)
    CBVideoActive: TCheckBox;
    Chart1: TChart;
    Edit1: TEdit;
    EditLine: TEdit;
    EditLineCount: TEdit;
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
    procedure qsort(var a : TMaxColorArray);
  private
    { private declarations }
  public
    { public declarations }
    FrameRate: integer;
    FrameTime : DWord;
  end;

var
  FMain: TFMain;
  NetOutBuf: TUDPBuffer;
  max_index_array: TMaxColorArray;
const
  MIN_Color = 90;
implementation

{ TFMain }

procedure TFMain.SendCameraMessage(mtype: integer);
var i: integer;
    ld: string;
begin
    ClearUDPBuffer(NetOutBuf);
    NetPutByte(NetOutBuf, ord('1'));
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


    UDPRob.Send(NetOutBuf.data, NetOutBuf.MessSize, '127.0.0.1:9020');
end;

procedure TFMain.FormDestroy(Sender: TObject);
begin
  UDPRob.Disconnect();
end;



procedure TFMain.qsort(var a : TMaxColorArray);

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

procedure TFMain.VideoFrame(Sender: TObject; FramePtr: PByte);
var lineCount, line, i, j, k, r, g, b: integer;
    BGR24Pixel: TBGR24Pixel;
    max_red_array, max_blue_array,max_green_array: TMaxColorArray; // Organizado 1º RGB depois faz sort por .Index
    Rarray, Garray, Barray: TMaxColorArray;
    sum_color: TIntArray;
    Near_Pixel : Boolean;
    sum_color_size: Integer;
    min_index: Integer;
    // aux_color_array : TMaxColorArray;
   // aux_color : TMaxColor;
    //ir, ig, ib : Integer;
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

  LineSeriesBlue.Clear;
  LineSeriesRED.Clear;
  LineSeriesGreen.Clear;
  line := StrToIntDef(EditLine.Text, 100);
  lineCount := StrToIntDef(EditLineCount.Text, 8);

  SetLength(max_red_array,0);
  SetLength(max_green_array,0);
  SetLength(max_blue_array,0);
  SetLength(Rarray, 0);
  SetLength(Garray, 0);
  SetLength(Barray, 0);
  SetLength(sum_color, 0);
  SetLength(max_index_array, 5);


  for i := 0 to 4 do begin
      max_index_array[i].Max := 0;
      max_index_array[i].Index := 0;
      max_index_array[i].Color := 'NULL';
  end;

  for i := 0 to Video.Width - 1 do begin
    r := 0;
    g := 0;
    b := 0;
    for j := 0 to lineCount do begin
      BGR24Pixel := pBGR24Pixel(FramePtr + ((line + j) * Video.Width  + i) * sizeof(BGR24Pixel))^;
      r := r + BGR24Pixel.Red;
      g := g + BGR24Pixel.Green;
      b := b + BGR24Pixel.Blue;
    end;

    SetLength(sum_color, Length(sum_color)+1);
    SetLength(Rarray, Length(Rarray)+1);
    SetLength(Garray, Length(Garray)+1);
    SetLength(Barray, Length(Barray)+1);

    sum_color_size:=Length(sum_color)-1;

    sum_color[sum_color_size] := 0;

    if(r > 200) then begin
      SetLength(max_red_array, Length(max_red_array)+1);
      max_red_array[Length(max_red_array)-1].Index := i;
      max_red_array[Length(max_red_array)-1].Max := r;
      max_red_array[Length(max_red_array)-1].Color := 'r';
      sum_color[sum_color_size] := sum_color[sum_color_size] + r;
      Rarray[sum_color_size] :=   max_red_array[Length(max_red_array)-1];
    end else begin
      Rarray[sum_color_size].Max := 0;
    end;

    if(g > 105) then begin
      SetLength(max_green_array, Length(max_green_array)+1);
      max_green_array[Length(max_green_array)-1].Index := i;
      max_green_array[Length(max_green_array)-1].Max := g;
      max_green_array[Length(max_green_array)-1].Color := 'g';
      sum_color[sum_color_size] := sum_color[sum_color_size] + g;
      Garray[sum_color_size] :=   max_green_array[Length(max_green_array)-1];
    end else begin
      Garray[sum_color_size].Max := 0;
    end;

    if(b > 100) then begin
      SetLength(max_blue_array, Length(max_blue_array)+1);
      max_blue_array[Length(max_blue_array)-1].Index := i;
      max_blue_array[Length(max_blue_array)-1].Max := b;
      max_blue_array[Length(max_blue_array)-1].Color := 'b';
      sum_color[sum_color_size] := sum_color[sum_color_size] + b;
      Barray[sum_color_size] :=   max_blue_array[Length(max_blue_array)-1];
    end else begin
      Barray[sum_color_size].Max := 0;
    end;

    LineSeriesRED.AddXY(i, r);
    LineSeriesGreen.AddXY(i, g);
    LineSeriesBlue.AddXY(i, b);
  end;

  Memo.Lines.Clear;
  Memo.Lines.add(floattostr(Length(max_blue_array)) + ' r' + floattostr(Length(max_red_array)) + ' g'+ floattostr(Length(max_green_array)));
  min_index := 0;
  Near_Pixel:=false;
  for i:=0 to Length(sum_color) - 1 do begin

      min_index := 0;

      for j:=1 to 4 do begin
          if(max_index_array[j].Max < max_index_array[j-1].Max) then begin
              min_index:=j;
          end;
      end;

      if(sum_color[i] > max_index_array[min_index].Max) then begin

         for k:= 0 to 4 do begin
              if (abs(max_index_array[k].Index - i) < 10) and (max_index_array[k].Index<>i) then begin
                  Near_Pixel:=true;   break;
              end;
          end;
         if not(Near_Pixel) then begin
              max_index_array[min_index].Max := sum_color[i];
              max_index_array[min_index].Index := i;
         end;
      end;
      Near_Pixel:=false;
      min_index := 0;
  end;

  (*for i:= 0 to 4 do begin
       for j:= 0 to 4 do begin
         if(abs(max_index_array[i].Index - max_index_array[j].Index) < 10)
         and (max_index_array[i].Max <= max_index_array[j].Max ) and (i<>j) then begin
             max_index_array[i].Max := 0;
             max_index_array[i].Index := 0;
         end;
     end;
  end;*)
    {
  for i:=0 to 4 do begin
      Memo.Lines.add(inttostr(max_index_array[i].Index)+' MAX '+inttostr(max_index_array[i].Max)+' Col '+max_index_array[i].Color);
  end;
     }

   for i:= 0 to 4 do begin
      if (Rarray[max_index_array[i].Index].Max > Garray[max_index_array[i].Index].Max) and
         (Rarray[max_index_array[i].Index].Max > Barray[max_index_array[i].Index].Max) then begin
           max_index_array[i]:= Rarray[max_index_array[i].Index]; Continue;
      end;
      if (Garray[max_index_array[i].Index].Max > Rarray[max_index_array[i].Index].Max) and
         (Garray[max_index_array[i].Index].Max > Barray[max_index_array[i].Index].Max) then begin
           max_index_array[i]:= Garray[max_index_array[i].Index]; Continue;
      end;
      if (Barray[max_index_array[i].Index].Max > Garray[max_index_array[i].Index].Max) and
         (Barray[max_index_array[i].Index].Max > Rarray[max_index_array[i].Index].Max) then begin
           max_index_array[i]:= Barray[max_index_array[i].Index]; Continue;
      end;
   end;

  qsort(max_index_array);

  Memo.Lines.add('###################################');

  for i:=0 to 4 do begin
      Memo.Lines.add(inttostr(max_index_array[i].Index)+' MAX '+inttostr(max_index_array[i].Max)+' Col '+max_index_array[i].Color);
  end;

  Memo.Lines.add('###################################');

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

