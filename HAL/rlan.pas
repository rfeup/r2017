unit RLan;

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Classes, Graphics, Controls,
  StdCtrls, ExtCtrls, math;

const
  UDPBufSize=1500;

type

  TUDPBuffer= record
    data: array[0..UDPBufSize-1] of byte;
    MessSize, ReadDisp: integer;
  end;

procedure AddToUDPBuffer(var Buf: TUDPBuffer; it: pointer; size_it: integer);
procedure ClearUDPBuffer(var Buf: TUDPBuffer);

procedure NetPutBuffer(var Buf: TUDPBuffer; var data; size_it: integer);
procedure NetBufferSeek(var Buf: TUDPBuffer; disp: integer);
procedure NetGetBuffer(var Buf: TUDPBuffer; var data; size_it: integer);

procedure NetPutByte(var Buf: TUDPBuffer; value: byte);
procedure NetPutWord(var Buf: TUDPBuffer; value: word);
procedure NetPutShort(var Buf: TUDPBuffer; value: SmallInt);
procedure NetPutInt(var Buf: TUDPBuffer; value: integer);
procedure NetPutFloat(var Buf: TUDPBuffer; value: single);
procedure NetPutString(var Buf: TUDPBuffer; str: string);
procedure NetPutAngle(var Buf: TUDPBuffer; value: double);

function NetPeekByte(var Buf: TUDPBuffer): byte;
function NetGetByte(var Buf: TUDPBuffer): byte;
function NetGetWord(var Buf: TUDPBuffer): word;
function NetGetShort(var Buf: TUDPBuffer): SmallInt;
function NetGetInt(var Buf: TUDPBuffer): integer;
function NetGetFloat(var Buf: TUDPBuffer): single;
function NetGetString(var Buf: TUDPBuffer): string;
function NetGetAngle(var Buf: TUDPBuffer): double;


function isValidMessage(mess: string): boolean;
function ExtractMessage(var sbuf: string): string;
procedure Hex2Raw(txt: string; var raw_buf: array of byte);

implementation

//const
//  digstohexdigs: array[0..10] of integer=(0,2,2,3,4,5,6,7,7,8,8);

procedure NetPutBuffer(var Buf: TUDPBuffer; var data; size_it: integer);
begin
  if Buf.MessSize+size_it >= UDPBufSize then exit;
  //copyMemory(@(Buf.data[Buf.MessSize]),@data,size_it);
  move(data, Buf.data[Buf.MessSize], size_it);
  Buf.MessSize:=Buf.MessSize+size_it;
end;

procedure NetBufferSeek(var Buf: TUDPBuffer; disp: integer);
begin
  if (disp>Buf.MessSize) or (disp=-1) then disp:=Buf.MessSize;
  Buf.ReadDisp:=disp;
end;

procedure NetGetBuffer(var Buf: TUDPBuffer; var data; size_it: integer);
begin
  if Buf.ReadDisp+size_it>Buf.MessSize then exit;
  //copyMemory(@data,@(Buf.data[Buf.ReadDisp]),size_it);
  move(Buf.data[Buf.ReadDisp], data, size_it);
  Buf.ReadDisp:=Buf.ReadDisp+size_it;
end;


procedure NetPutByte(var Buf: TUDPBuffer; value: byte);
begin
  NetPutBuffer(Buf,value,sizeof(value));
end;

procedure NetPutWord(var Buf: TUDPBuffer; value: word);
begin
  NetPutBuffer(Buf,value,sizeof(value));
end;

procedure NetPutShort(var Buf: TUDPBuffer; value: SmallInt);
begin
  NetPutBuffer(Buf,value,sizeof(value));
end;

procedure NetPutInt(var Buf: TUDPBuffer; value: integer);
begin
  NetPutBuffer(Buf,value,sizeof(value));
end;

procedure NetPutFloat(var Buf: TUDPBuffer; value: single);
begin
  NetPutBuffer(Buf,value,sizeof(value));
end;

procedure NetPutString(var Buf: TUDPBuffer; str: string);
var len: word;
begin
  len:=length(str);
  //NetPutBuffer(Buf,len,sizeof(len));
  NetPutWord(Buf,len);
  if len>0 then
    NetPutBuffer(Buf,str[1],len);
end;

procedure NetPutAngle(var Buf: TUDPBuffer; value: double);
var tmp: word;
begin
//  tmp:=round((NormalizeAngle(value)+pi)*10000);
//  NetPutWord(Buf,tmp);
end;

function NetPeekByte(var Buf: TUDPBuffer): byte;
begin
  result:=0;
  if Buf.ReadDisp>Buf.MessSize then exit;
  result:=Buf.data[Buf.ReadDisp];
end;

function NetGetByte(var Buf: TUDPBuffer): byte;
begin
  NetGetBuffer(Buf,result,sizeof(result));
end;

function NetGetWord(var Buf: TUDPBuffer): word;
begin
  NetGetBuffer(Buf,result,sizeof(result));
end;

function NetGetShort(var Buf: TUDPBuffer): SmallInt;
begin
  NetGetBuffer(Buf,result,sizeof(result));
end;

function NetGetInt(var Buf: TUDPBuffer): integer;
begin
  NetGetBuffer(Buf,result,sizeof(result));
end;

function NetGetFloat(var Buf: TUDPBuffer): single;
begin
  NetGetBuffer(Buf,result,sizeof(result));
end;

function NetGetString(var Buf: TUDPBuffer): string;
var size: word;
begin
  result:='';
  size:=NetGetWord(Buf);
  if size=0 then exit;
  result:=stringofchar(chr(0),size);
  NetGetBuffer(Buf,result[1],size);
end;

function NetGetAngle(var Buf: TUDPBuffer): double;
begin
//  result:=NormalizeAngle(NetGetWord(buf)/10000-pi);
end;



function ExtractMessage(var sbuf: string): string;
var i: integer;
begin
  result:='';
  while (sbuf<>'') do begin
    if sbuf[1]<>'<' then begin
      sbuf:=copy(sbuf,2,length(sbuf));
    end else begin
      break;
    end;
  end;

  if (sbuf<>'') and (sbuf[1]='<') then begin
    i:=2;
    while i<=length(sbuf) do begin
      if sbuf[i]='<' then begin
        sbuf:=copy(sbuf,i,length(sbuf));
        i:=2;
      end else if sbuf[i]='>' then begin
        result:=copy(sbuf,1,i);
        sbuf:=copy(sbuf,i+1,length(sbuf));
      end else begin
        inc(i);
      end;
    end;
  end;

end;

function isValidMessage(mess: string): boolean;
var i: integer;
begin
  result:=false;
  if mess='' then exit;
  if mess[1]<>'<' then exit;
  if mess[length(mess)]<>'>' then exit;
  for i:=2 to length(mess)-1 do begin
    if not (mess[i] in ['0'..'9','A'..'F','a'..'f']) then exit;
  end;
  result:=true;
end;

procedure Hex2Raw(txt: string; var raw_buf: array of byte);
var i: integer;
begin
  for i:=0 to (length(txt) div 2)-1 do begin
    raw_buf[i]:=StrToInt('$'+copy(txt,i*2+1,2));
  end;
end;


procedure ClearUDPBuffer(var Buf:TUDPBuffer);
begin
  //zeroMemory(@(Buf.data[0]),UDPBufSize);
  fillbyte(Buf.data, sizeof(Buf.data), 0);
  Buf.MessSize:=0;
end;

procedure AddToUDPBuffer(var Buf:TUDPBuffer; it: pointer; size_it: integer);
begin
  if Buf.MessSize+size_it >= UDPBufSize then exit;
  //copyMemory(@(Buf.data[Buf.MessSize]),it,size_it);
  move(it, Buf.data[Buf.MessSize], size_it);
  Buf.MessSize:=Buf.MessSize+size_it;
end;


end.
