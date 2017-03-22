unit Bitmap_grid;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dynmatrix, dynmatrixutils, BGRABitmap, BGRABitmapTypes;

procedure DrawMap(var bmp:TBGRABitmap; Width, Height, Sizec, Dist_center, Width_maq, Height_maq: double );
procedure BitmapToMatrix(var map : TDMatrix; bmp : TBGRABitmap);
procedure MatrixToBitmap(var bmp : TBGRABitmap; map : TDMatrix; Obst : integer);

implementation

procedure DrawMap(var bmp:TBGRABitmap; Width, Height, Sizec, Dist_center, Width_maq, Height_maq: double );
begin
  bmp.Rectangle(0,0,round(Width/Sizec),round(Height/Sizec),BGRA(255,0,0), dmset);

  bmp.FillRect(round((Width/2-Dist_center-Width_maq)/Sizec),
               round((Height/2-Height_maq/2)/Sizec),
               round((Width/2-Dist_center)/Sizec),
               round((Height/2+Height_maq/2)/Sizec),BGRA(255,0,0), dmSet);

  bmp.FillRect(round((Width/2+Dist_center)/Sizec),
               round((Height/2-Height_maq/2)/Sizec),
               round((Width/2+Dist_center+Width_maq)/Sizec),
               round((Height/2+Height_maq/2)/Sizec),BGRA(255,0,0), dmSet);


end;


procedure BitmapToMatrix(var map : TDMatrix; bmp : TBGRABitmap);
var
PPixel : PBGRAPixel;
y,x,v : Integer;
begin
  map.setsize(bmp.Height,bmp.Width);
  PPixel := bmp.Data;
  for y := 0 to bmp.Height - 1 do begin
    for x := 0 to bmp.Width - 1 do begin
      if(PPixel^.blue = 0) and (PPixel^.red = 255) and (PPixel^.green = 0) then begin
         map.setv(y,x,1);
      end else
      begin
         map.setv(y,x,0);
      end;
      inc(PPixel);
    end;
  end;
end;

procedure MatrixToBitmap(var bmp : TBGRABitmap; map : TDMatrix; Obst : integer);
var
PPixel : PBGRAPixel;
y,x,v : Integer;
begin

  PPixel := bmp.Data;

  for y := 0 to map.NumRows - 1 do begin
    for x := 0 to map.NumCols - 1 do begin
      if(map.getv(y,x) = 0) then begin
      PPixel^.red:=0;
      PPixel^.blue:=0;
      PPixel^.green:=255;
      end else if(map.getv(y,x) = obst) then begin
      PPixel^.red:=255;
      PPixel^.blue:=0;
      PPixel^.green:=0;
      end else if(map.getv(y,x) = 3) then begin
      PPixel^.red:=255;
      PPixel^.blue:=0;
      PPixel^.green:=165;
      end else if(map.getv(y,x) = 2) then begin
      PPixel^.red:=255;
      PPixel^.blue:=0;
      PPixel^.green:=215;
      end else if(map.getv(y,x) = 1) then begin
      PPixel^.red:=255;
      PPixel^.blue:=0;
      PPixel^.green:=255;
      end;
      inc(PPixel);
    end;
  end;

end;

end.

