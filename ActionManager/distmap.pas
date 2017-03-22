unit DistMap;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dynmatrix, dynmatrixutils;
Const
  Inf = 1.0/0.0;


function DistMapMat(Mapa:TDMatrix; d1,d2 : Double) : TDMatrix;

implementation

function DistMapMat(Mapa:TDMatrix; d1,d2 : Double) : TDMatrix;
Var
  i,j :integer;
  Dist : TDMatrix;
  Value : TDMatrix;
begin
Dist := Mapa;
Value.SetSize(1,5);

//Forward
for i := 1 to Mapa.NumRows-1 do
begin
  for j := 1 to Mapa.NumCols-1 do
  begin
    if (dist.ugetv(i-1,j-1) = 0) then begin
        dist.usetv(i-1,j-1,Inf);
    end
    else if (dist.ugetv(i-1,j) = 0) then begin
        dist.usetv(i-1,j,Inf);
    end
    else if (dist.ugetv(i-1,j+1) = 0) then begin
        dist.usetv(i-1,j+1,Inf);
    end
    else if (dist.ugetv(i,j-1) = 0) then begin
        dist.usetv(i,j-1,Inf);
    end
    else if (dist.ugetv(i,j) = 0) then begin
        dist.usetv(i,j,Inf);
    end;

    Value.setv(0,0,dist.ugetv(i-1,j-1)+d2);
    Value.setv(0,1,dist.ugetv(i-1,j)+d1);
    Value.setv(0,2,dist.ugetv(i-1,j+1)+d2);
    Value.setv(0,3,dist.ugetv(i,j-1)+d1);
    Value.setv(0,4,dist.ugetv(i,j));

    dist.setv(i,j,Mmin(Value));

  end;
end;

//Backward
for i := Mapa.NumRows-2 downto 0 do
begin
  for j := Mapa.NumCols-2 downto 0 do
  begin

    Value.setv(0,0,dist.ugetv(i+1,j+1)+d2);
    Value.setv(0,1,dist.ugetv(i+1,j)+d1);
    Value.setv(0,2,dist.ugetv(i+1,j-1)+d2);
    Value.setv(0,3,dist.ugetv(i,j+1)+d1);
    Value.setv(0,4,dist.ugetv(i,j));

    dist.setv(i,j,Mmin(Value));

  end;
end;

Result := Dist;

end;

end.

