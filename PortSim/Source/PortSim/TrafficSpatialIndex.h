#pragma once
#include "CoreMinimal.h"

// Broad phase only: callers retain their exact 3D envelope intersection tests.
// Closed XY cells include touching boundaries and negative world coordinates.
class FTrafficSpatialIndex
{
public:
    static constexpr double CellSize=2000.;
    void Reset() { Cells.Reset(); Membership.Reset(); }
    void Remove(int32 ID)
    {
        const auto* Previous=Membership.Find(ID);
        if (!Previous) return;
        for (const FIntPoint& Cell:*Previous)
            if (auto* IDs=Cells.Find(Cell))
            {
                IDs->Remove(ID);
                if (IDs->IsEmpty()) Cells.Remove(Cell);
            }
        Membership.Remove(ID);
    }
    void Set(int32 ID,const TArray<FBox>& Bounds)
    {
        TArray<FIntPoint> Keys;
        for (const FBox& Box:Bounds) Visit(Box,[&](FIntPoint Cell) { Keys.AddUnique(Cell); });
        if (const auto* Previous=Membership.Find(ID); Previous && *Previous==Keys) return;
        Remove(ID);
        for (const FIntPoint& Cell:Keys) Cells.FindOrAdd(Cell).Add(ID);
        Membership.Add(ID,MoveTemp(Keys));
    }
    TSet<int32> Query(const TArray<FBox>& Bounds) const
    {
        TSet<int32> Result;
        for (const FBox& Box:Bounds) Visit(Box,[&](FIntPoint Cell)
        {
            if (const auto* IDs=Cells.Find(Cell)) Result.Append(*IDs);
        });
        return Result;
    }
private:
    template<typename F> static void Visit(const FBox& Box,F&& Visitor)
    {
        if (!Box.IsValid) return;
        const int32 MinX=FMath::FloorToInt(Box.Min.X/CellSize), MaxX=FMath::FloorToInt(Box.Max.X/CellSize);
        const int32 MinY=FMath::FloorToInt(Box.Min.Y/CellSize), MaxY=FMath::FloorToInt(Box.Max.Y/CellSize);
        for (int32 X=MinX;X<=MaxX;++X) for (int32 Y=MinY;Y<=MaxY;++Y) Visitor(FIntPoint(X,Y));
    }
    TMap<FIntPoint,TSet<int32>> Cells;
    TMap<int32,TArray<FIntPoint>> Membership;
};
