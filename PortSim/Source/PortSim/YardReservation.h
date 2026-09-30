#pragma once
#include "CoreMinimal.h"

// Choose complete stacks so no occupied tier is left above an empty receiving slot.
inline bool SelectWholeYardStacks(const TArray<int32>& Sizes,int32 Capacity,TArray<int32>& Selected)
{
    Selected.Reset();
    if (Capacity<0) return false;
    TArray<int32> Previous;
    Previous.Init(INDEX_NONE,Capacity+1); Previous[0]=-2;
    for (int32 Stack=0;Stack<Sizes.Num() && Previous[Capacity]==INDEX_NONE;++Stack)
    {
        const int32 Count=Sizes[Stack];
        if (Count<=0) continue;
        for (int32 N=Capacity;N>=Count;--N)
            if (Previous[N]==INDEX_NONE && Previous[N-Count]!=INDEX_NONE) Previous[N]=Stack;
    }
    if (Previous[Capacity]==INDEX_NONE) return false;
    for (int32 Remaining=Capacity;Remaining>0;)
    {
        const int32 Stack=Previous[Remaining];
        Selected.Add(Stack); Remaining-=Sizes[Stack];
    }
    return true;
}
