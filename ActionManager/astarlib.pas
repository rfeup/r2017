unit Astarlib;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, dynmatrix, BinHeap;
const
  EXTRA = 20;

function Neighbors(Map:TDMatrix; Parent: PTNode; Obstacle: Double; Sizec : double; Row,Col : integer): TNodeArray;
function Contains(Heap : TNodeArray; var Node_comp : TNode) : Integer;
function Euclidean_distance(X_node, X_goal, Y_node, Y_goal:double): double;
function Astar(Map: TDMatrix; Start, Goal : TNode; Sizec, Obstacle:double) : TNodeArray;

implementation

function Neighbors(Map:TDMatrix; Parent: PTNode; Obstacle: Double; Sizec : double; Row,Col : integer): TNodeArray;
Var
  Node : TNodeArray ;
  Aux : Double;
begin
     SetLength(Node, 8);

     //North

     Node[0].X := Col;
     Node[0].Y := Row+1;
     Node[0].cost := Sizec;
     Node[0].Parent := Parent;

     Aux := Map.ugetv(Row+1,Col);

     (*if ( Aux = 0) and
        (Row +1 < Map.NumRows) then  begin
         Node[0].Passable := True;
         Node[0].Near_obstacle := False;
     end else *)if (Aux = Obstacle)or
        (Row +1 >= Map.NumRows) then  begin
         Node[0].Passable := False;
         Node[0].Near_obstacle := False;
     end else begin
         Node[0].Passable := True;
         Node[0].Near_obstacle := True;
         Node[0].cost := Node[0].cost+Node[0].cost*Aux;
     end;

     //Northwest

     Node[1].X := Col-1;
     Node[1].Y := Row+1;
     Node[1].cost := Sizec*sqrt(2);
     Node[1].Parent := Parent;

     Aux := Map.ugetv(Row+1,Col-1);
     (*if (Aux = 0) and
        (Row +1 < Map.NumRows) and
        (Col-1 >= 0) then
     begin
         Node[1].Passable := True;
         Node[1].Near_obstacle := False;
     end
     else *)if (Aux = Obstacle) or
        ((Row +1 >= Map.NumRows) or
        (Col-1 < 0)) then
     begin
          Node[1].Passable := False;
          Node[1].Near_obstacle := False;
     end
     else
     begin
          Node[1].Passable := True;
          Node[1].Near_obstacle := True;
          Node[1].cost := Node[1].cost+Node[1].cost*Aux;
     end;

     //West

     Node[2].X := Col-1;
     Node[2].Y := Row;
     Node[2].cost := Sizec;
     Node[2].Parent := Parent;

     Aux := Map.ugetv(Row,Col-1);
     (*if (Aux = 0)  and
        (Col-1 >= 0) then
     begin
     Node[2].Passable := True;
     Node[2].Near_obstacle := False;
     end
     else *)if (Aux = Obstacle) or
        (Col-1 < 0) then
     begin
     Node[2].Passable := False;
     Node[2].Near_obstacle := False;
     end
     else
     begin
     Node[2].Passable := True;
     Node[2].Near_obstacle := True;
     Node[2].cost := Node[2].cost+Node[2].cost*Aux;
     end;

     //Southwest

     Node[3].X := Col-1;
     Node[3].Y := Row-1;
     Node[3].cost := Sizec*sqrt(2);
     Node[3].Parent := Parent;

     Aux := Map.ugetv(Row-1,Col-1);
     (*if (Aux = 0)  and
        (Row -1 >= 0) and
        (Col-1 >= 0) then
     begin
     Node[3].Passable := True;
     Node[3].Near_obstacle := False;
     end
     else*) if (Aux = Obstacle) or
        ((Row -1 < 0) or
        (Col-1 < 0)) then
     begin
     Node[3].Passable := False;
     Node[3].Near_obstacle := False;
     end
     else
     begin
     Node[3].Passable := True;
     Node[3].Near_obstacle := True;
     Node[3].cost := Node[3].cost+Node[3].cost*Aux;
     end;

     //South

     Node[4].X := Col;
     Node[4].Y := Row-1;
     Node[4].cost := Sizec;
     Node[4].Parent := Parent;

     Aux := Map.ugetv(Row-1,Col);
     (*if (Aux = 0) and
        (Row -1 >=0)   then
     begin
     Node[4].Passable := True;
     Node[4].Near_obstacle := False;
     end
     else *)if (Aux = Obstacle) or
        (Row-1<0)  then
     begin
     Node[4].Passable := False;
     Node[4].Near_obstacle := False;
     end
     else
     begin
     Node[4].Passable := True;
     Node[4].Near_obstacle := True;
     Node[4].cost := Node[4].cost+Node[4].cost*Aux;
     end;

     //Southeast

     Node[5].X := Col+1;
     Node[5].Y := Row-1;
     Node[5].cost := Sizec*sqrt(2);
     Node[5].Parent := Parent;

     Aux := Map.ugetv(Row-1,Col+1);
     (*if (Aux = 0) and
        (Row -1>=0) and
        (Col+1 < Map.NumCols)  then
     begin
     Node[5].Passable := True;
     Node[5].Near_obstacle := False;
     end
     else *)if (Aux = Obstacle)or
        ((Row-1<0) or
        (Col+1 >= Map.NumCols)) then
     begin
     Node[5].Passable := False;
     Node[5].Near_obstacle := False;
     end
     else
     begin
     Node[5].Passable := True;
     Node[5].Near_obstacle := True;
     Node[5].cost := Node[5].cost+Node[5].cost*Aux;
     end;

     //East

     Node[6].X := Col+1;
     Node[6].Y := Row;
     Node[6].cost := Sizec;
     Node[6].Parent := Parent;

     Aux := Map.ugetv(Row,Col+1);
     (*if (Aux = 0)and
        (Col+1 < Map.NumCols)  then
     begin
     Node[6].Passable := True;
     Node[6].Near_obstacle := False;
     end
     else*) if (Aux = Obstacle)or
        (Col+1 >= Map.NumCols) then
     begin
     Node[6].Passable := False;
     Node[6].Near_obstacle := False;
     end
     else
     begin
     Node[6].Passable := True;
     Node[6].Near_obstacle := True;
     Node[6].cost := Node[6].cost+Node[6].cost*Aux;
     end;

     //Northeast

     Node[7].X := Col+1;
     Node[7].Y := Row+1;
     Node[7].cost := Sizec*sqrt(2);
     Node[7].Parent := Parent;

     Aux := Map.ugetv(Row+1,Col+1);
    (* if (Aux = 0)and
        (Row +1<Map.NumRows) and
        (Col+1 < Map.NumCols)  then
     begin
     Node[7].Passable := True;
     Node[7].Near_obstacle := False;
     end
     else *)if (Aux = Obstacle)or
        ((Row +1 >= Map.NumRows) or
        (Col+1 >= Map.NumCols))then
     begin
     Node[7].Passable := False;
     Node[7].Near_obstacle := False;
     end
     else
     begin
     Node[7].Passable := True;
     Node[7].Near_obstacle := True;
     Node[7].cost := Node[7].cost+Node[7].cost*Aux;
     end;

     result:= Node;
end;

function Contains(Heap : TNodeArray; var Node_comp : TNode) : Integer;
var
  i : integer;
  index : integer = -1;
begin

     for i := 0 to Length(Heap)-1 do begin
         if (Node_comp.X = Heap[i].X) and (Node_comp.Y = Heap[i].Y) then begin
             index := i;
             Node_comp.g := Heap[i].g;
             Break;
         end;
     end;

     Result := index;

end;

function Euclidean_distance(X_node, X_goal, Y_node, Y_goal:double): double;
var
     dX,dY : double;
     D : double = 1;
     K : double = 1.3;
begin
      dX := abs(X_node - X_goal);
      dY := abs(Y_node - Y_goal);

      result := K * D* sqrt(dX*dX + dY*dY);
end;

function Astar(Map: TDMatrix; Start, Goal : TNode; Sizec, Obstacle:double) : TNodeArray;
Var
  Open_Heap, Neighbor, Visited, Res :TNodeArray;
  Curr_Node: TNode;
  i, pos, isOpen, isClose: integer;
  Par_Ptr, Aux: PTNode;
begin
  Start.Open:=True;
  Start.Closed:=False;
  Start.Near_obstacle:=False;
  Start.g:=0;
  Start.h := Euclidean_distance(Sizec/2 + Sizec*(Start.X), Sizec/2 + Sizec*(Goal.X),
                                Sizec/2 + Sizec*(Start.Y), Sizec/2 + Sizec*(Goal.Y));
  Start.f := Start.g + Start.h;
  Start.Parent := nil;
  add(Open_Heap,Start);

  While Length(Open_Heap) > 0 do begin

        Curr_Node := remove(Open_heap);
        Curr_Node.Closed := True;
        Curr_Node.Open := False;

        SetLength(Visited, Length(Visited)+1);
        pos := Length(Visited);
        Visited[pos-1] := Curr_Node;

        if (Curr_Node.X = Goal.X) and (Curr_Node.Y = Goal.Y) then begin

             Aux := @Visited[pos-1];
             while (Aux <> nil) do begin
                  SetLength(Res, Length(Res)+1);
                  Res[Length(Res)-1] := Aux^;
                  Aux := Aux^.Parent;
             end;

             Result := Res;

             Exit;

        end;

        new(Par_Ptr);
        Par_Ptr^ := Visited[pos-1];

        Neighbor := Neighbors(Map,Par_Ptr, Obstacle, Sizec, Curr_Node.Y, Curr_node.X);

        for i := 0 to Length(Neighbor)-1 do begin

              if (Neighbor[i].Passable = False) then Continue;

              isOpen := Contains(Open_Heap, Neighbor[i]);
              if (isOpen>-1) then begin
                   Neighbor[i].Open := True;
                   Neighbor[i].Closed := False;
              end;

              isClose := Contains(Visited, Neighbor[i]);
              if (isClose>-1) then begin
                   Neighbor[i].Closed := True;
                   Neighbor[i].Open := False;
              end;

              if (Neighbor[i].Open = False) and (Neighbor[i].Closed = False) then
              begin
                   Neighbor[i].g := Visited[pos-1].g + Neighbor[i].Cost;
                   Neighbor[i].h := Euclidean_distance(Sizec/2 + Sizec*(Neighbor[i].X), Sizec/2 + Sizec*(Goal.X),
                                                       Sizec/2 + Sizec*(Neighbor[i].Y), Sizec/2 + Sizec*(Goal.Y));
                   Neighbor[i].f := Neighbor[i].g + Neighbor[i].h;

                   add(Open_Heap,Neighbor[i]);

                   Continue;
              end;

              if (Neighbor[i].Open = True) and
                 ( Visited[pos-1].g + Neighbor[i].Cost < Open_Heap[isOpen].g) then begin

                      Open_Heap[isOpen].g := Visited[pos-1].g + Neighbor[i].Cost;
                      Open_Heap[isOpen].f := Open_Heap[isOpen].g + Open_Heap[isOpen].h;
                      Open_Heap[isOpen].Parent := Par_Ptr;//@Visited[pos-1];
                      BubbleUp(Open_Heap, isOpen);
                      Continue;

              end;

              //if (Neighbor[i].Closed = True) and
              //   ( Visited[pos-1].g + Neighbor[i].Cost < Visited[isClose].g) then begin
              //
              //        Visited[isClose].g := Visited[pos-1].g + Neighbor[i].Cost;
              //        Visited[isClose].f := Visited[isClose].g +  Visited[isClose].h;
              //        Visited[isClose].Parent := Par_Ptr;//@Visited[pos-1];
              //        Visited[isClose].Closed := False;
              //        Visited[isClose].Open := True;
              //
              //        add(Open_Heap, Visited[isClose]);
              //
              //        Visited[isClose].X := -1;
              //        Visited[isClose].Y := -1;
              //        Continue;
              //
              //end;
        end;

  end;

end;

end.

