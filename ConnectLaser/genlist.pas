unit genlist;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils;


type
  { TGenericList }

  generic TGenericList<_TGenItem> = class(TList)
    var
    private
    protected
      function GetItems(Index: Integer): _TGenItem;
      procedure SetItems(Index: Integer; AItem: _TGenItem);
    public
      function Add(AItem: _TGenItem): Integer;
      function Extract(AItem: _TGenItem): _TGenItem;
      function Remove(AItem: _TGenItem): Integer;
      function IndexOf(AItem: _TGenItem): Integer;
      function First: _TGenItem;
      function Last: _TGenItem;
      procedure Insert(Index: Integer; AItem: _TGenItem);
      property Items[Index: Integer]: _TGenItem read GetItems write SetItems; default;
      procedure FreeAllItems;
  end;

implementation

{ TGenericList }

function TGenericList.GetItems(Index: Integer): _TGenItem;
begin
  Result := _TGenItem(inherited Items[Index]);
end;

procedure TGenericList.SetItems(Index: Integer; AItem: _TGenItem);
begin
  inherited Items[Index] := AItem;
end;

function TGenericList.Add(AItem: _TGenItem): Integer;
begin
  Result := inherited Add(AItem);
end;

function TGenericList.Extract(AItem: _TGenItem): _TGenItem;
begin
  Result := _TGenItem(inherited Extract(AItem));
end;

function TGenericList.Remove(AItem: _TGenItem): Integer;
begin
  Result := inherited Remove(AItem);
end;

function TGenericList.IndexOf(AItem: _TGenItem): Integer;
begin
  Result := inherited IndexOf(AItem);
end;

function TGenericList.First: _TGenItem;
begin
  Result := _TGenItem(inherited First);
end;

function TGenericList.Last: _TGenItem;
begin
  Result := _TGenItem(inherited Last);
end;

procedure TGenericList.Insert(Index: Integer; AItem: _TGenItem);
begin
  inherited Insert(Index, AItem);
end;

procedure TGenericList.FreeAllItems;
var i: integer;
begin
  for i := 0 to count-1 do begin
    GetItems(i).Free;
  end;
  clear;
end;

end.

