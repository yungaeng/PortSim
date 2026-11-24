#pragma once
#include "CoreMinimal.h"

namespace YardCargoQuota
{
    // Largest-remainder apportionment, bounded by physical receiving capacity.
    inline bool Allocate(const TArray<int32>& Capacity,int32 Count,TArray<int32>& Quota)
    {
        Quota.Reset();
        int64 Total=0;
        for(int32 C:Capacity) { if(C<0) return false; Total+=C; }
        if(Count<0 || Count>Total) return false;
        Quota.Init(0,Capacity.Num());
        if(Count==0) return true;
        TArray<int64> Remainder; Remainder.Init(0,Capacity.Num());
        int32 Assigned=0;
        for(int32 I=0;I<Capacity.Num();++I)
        {
            const int64 Product=int64(Count)*Capacity[I];
            Quota[I]=int32(Product/Total); Remainder[I]=Product%Total;
            Assigned+=Quota[I];
        }
        while(Assigned<Count)
        {
            int32 Best=INDEX_NONE;
            for(int32 I=0;I<Capacity.Num();++I)
                if(Quota[I]<Capacity[I] && (Best==INDEX_NONE || Remainder[I]>Remainder[Best])) Best=I;
            if(Best==INDEX_NONE) return false;
            ++Quota[Best]; Remainder[Best]=-1; ++Assigned;
        }
        return true;
    }
}
