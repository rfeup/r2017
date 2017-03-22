unit BinHeap;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;

type

TPos = record
    X,Y: Integer;
end;

//TNode = record;
PTNode = ^TNode;

TNode = record
    X, Y : Integer;
    Cost : Double;
    f : Double;
    g : Double;
    h : Double;
    Passable : Boolean;
    Near_obstacle : Boolean;
    Visit : Boolean;
    Open : Boolean;
    Closed : Boolean;
    Parent : PTNode;
    Pos_Visit_Par : Integer;
end;


TNodeArray = array of TNode;

procedure swap(var Heap:TNodeArray; index1, index2:integer);
procedure BubbleUp(var Heap:TNodeArray; index:integer);
procedure BubbleDown(var Heap : TNodeArray);
procedure add(var Heap : TNodeArray; Value: TNode);
function remove(var Heap:TNodeArray) : TNode;

implementation

procedure swap(var Heap:TNodeArray; index1, index2:integer);
var
  aux:TNode;
begin

  aux := Heap[index1];
  Heap[index1] := Heap[index2];
  Heap[index2] := aux;

end;

procedure BubbleUp(var Heap:TNodeArray; index:integer);
begin

  while (index>0) and
        (Heap[trunc((index-1)/2)].f > Heap[index].f) do
  begin
       Swap(Heap,index, trunc((index-1)/2));
       index := trunc((index-1)/2);
  end;

end;

procedure BubbleDown(var Heap : TNodeArray);
var
  index : integer;
  smallerchild : integer;
begin
  index := 0;
  while index*2+1 < Length(Heap) do
  begin
       smallerchild := index*2+1;

       if (index*2+2 < Length(Heap)) and
          (Heap[index*2+1].f > Heap[index*2+2].f) then
       begin
         smallerchild:= index*2+2;
       end;

       if Heap[index].f > Heap[smallerchild].f then
       begin
         swap(Heap,index, smallerchild);
       end
       else
       begin
            Break;
       end;

       index := smallerchild;
  end;
end;

procedure add(var Heap : TNodeArray; Value: TNode);
Var
   index:integer;
begin

  SetLength(Heap, Length(Heap)+1);

  // insert element

  index := Length(Heap)-1;

  Heap[index] := value;

  BubbleUp(Heap,index);

end;

function remove(var Heap:TNodeArray) : TNode;
var
   index:integer;
begin
  result:=Heap[0];

  index:=Length(Heap)-1;

  Heap[0]:= Heap[index];
  SetLength(Heap, Length(Heap)-1);

  if (Length(Heap)>0) then
  begin
      BubbleDown(Heap);
  end;

end;

end.

