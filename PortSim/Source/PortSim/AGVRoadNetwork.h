#pragma once
#include "CoreMinimal.h"

// Directed, axis-aligned road centre lines. Endpoints and crossings are split
// into graph nodes; endpoints off the road never get a straight-line shortcut.
class FAGVRoadNetwork
{
public:
    struct FSegment { FVector A,B; bool Both; };
    TArray<FSegment> Roads;
    float NarrowEastY=MAX_flt,NarrowWestY=MAX_flt;
    void RestrictNarrowCrossing(float EastboundY,float WestboundY)
    { NarrowEastY=EastboundY; NarrowWestY=WestboundY; }
    void Add(FVector A,FVector B,bool Both=false)
    {
        if(!A.Equals(B,.1) && (FMath::IsNearlyEqual(A.X,B.X,.1) || FMath::IsNearlyEqual(A.Y,B.Y,.1)))
            Roads.Add({A,B,Both});
    }
    static bool Contains(const FSegment& S,FVector P)
    {
        // Sensor docking may stop a few hundredths of a centimetre past an
        // endpoint.  Treat the same 1 mm point tolerance as part of the road;
        // the normalized T tolerance alone shrinks on long terminal segments.
        if(P.Equals(S.A,.1f) || P.Equals(S.B,.1f)) return true;
        const FVector D=S.B-S.A;
        const double T=FVector::DotProduct(P-S.A,D)/D.SizeSquared();
        return T>=-1.e-6 && T<=1.+1.e-6 && (S.A+D*T).Equals(P,.1);
    }
    bool Find(FVector Start,FVector Goal,TArray<FVector>& Route,
        TFunctionRef<double(FVector,FVector)> Penalty) const
    {
        Route.Reset();
        if(Start.Equals(Goal,.1)) { Route.Add(Goal); return true; }
        TArray<FVector> Points;
        auto Node=[&](FVector P) { for(int32 I=0;I<Points.Num();++I) if(Points[I].Equals(P,.1)) return I; return Points.Add(P); };
        const int32 Source=Node(Start),Destination=Node(Goal);
        for(const auto& S:Roads)
        {
            Node(S.A); Node(S.B);
            const int32 Steps=FMath::CeilToInt(FVector::Dist2D(S.A,S.B)/3000.);
            for(int32 I=1;I<Steps;++I) Node(FMath::Lerp(S.A,S.B,double(I)/Steps));
        }
        for(int32 I=0;I<Roads.Num();++I) for(int32 J=I+1;J<Roads.Num();++J)
        {
            const auto& A=Roads[I]; const auto& B=Roads[J];
            const bool AV=FMath::IsNearlyEqual(A.A.X,A.B.X,.1), BV=FMath::IsNearlyEqual(B.A.X,B.B.X,.1);
            if(AV==BV) continue;
            const FVector P(AV?A.A.X:B.A.X,AV?B.A.Y:A.A.Y,0);
            if(Contains(A,P) && Contains(B,P)) Node(P);
        }
        struct FEdge { int32 To; double Cost; };
        TArray<TArray<FEdge>> Edges; Edges.SetNum(Points.Num());
        for(const auto& S:Roads)
        {
            TArray<int32> On;
            for(int32 I=0;I<Points.Num();++I) if(Contains(S,Points[I])) On.Add(I);
            On.Sort([&](int32 A,int32 B){ return FVector::DistSquared(S.A,Points[A])<FVector::DistSquared(S.A,Points[B]); });
            for(int32 I=1;I<On.Num();++I)
            {
                const int32 A=On[I-1],B=On[I];
                const double Length=FVector::Dist2D(Points[A],Points[B]);
                Edges[A].Add({B,Length+FMath::Max(0.,Penalty(Points[A],Points[B]))});
                if(S.Both) Edges[B].Add({A,Length+FMath::Max(0.,Penalty(Points[B],Points[A]))});
            }
        }
        // Remember entry from a narrow crossing until the vehicle clears its
        // opposite lane. Two same-side turns across that short gap cannot be
        // rounded at the real minimum radius. A node-only search loses this
        // history and repeatedly chooses an impossible U-turn.
        const bool Narrow=NarrowEastY<MAX_flt && NarrowWestY<MAX_flt;
        const int32 Modes=Narrow?4:1,StateCount=Points.Num()*Modes;
        auto InitialMode=[&]()
        {
            if(!Narrow) return 0;
            if(FMath::IsNearlyEqual(Start.Y,double(NarrowEastY),.1)) return 1;
            if(FMath::IsNearlyEqual(Start.Y,double(NarrowWestY),.1)) return 2;
            return Start.Y>NarrowWestY && Start.Y<NarrowEastY?3:0;
        };
        const int32 SourceState=Source*Modes+InitialMode();
        TArray<double> Distance; Distance.Init(TNumericLimits<double>::Max(),StateCount);
        TArray<int32> Previous; Previous.Init(INDEX_NONE,StateCount);
        TArray<bool> Closed; Closed.Init(false,StateCount); Distance[SourceState]=0;
        struct FOpen { int32 State; double Score; };
        TArray<FOpen> Open;
        auto Earlier=[](const FOpen& A,const FOpen& B){return A.Score<B.Score || (A.Score==B.Score && A.State<B.State);};
        auto Push=[&](int32 State,double Score)
        {
            int32 I=Open.Add({State,Score});
            while(I>0) { const int32 Parent=(I-1)/2; if(!Earlier(Open[I],Open[Parent])) break; Open.Swap(I,Parent); I=Parent; }
        };
        auto Pop=[&]()
        {
            const FOpen Result=Open[0]; Open[0]=Open.Last(); Open.Pop(EAllowShrinking::No);
            for(int32 I=0;I<Open.Num();)
            {
                int32 Child=I*2+1; if(Child>=Open.Num()) break;
                if(Child+1<Open.Num() && Earlier(Open[Child+1],Open[Child])) ++Child;
                if(!Earlier(Open[Child],Open[I])) break; Open.Swap(I,Child); I=Child;
            }
            return Result.State;
        };
        Push(SourceState,FVector::Dist2D(Start,Goal));
        while(!Open.IsEmpty())
        {
            const int32 BestState=Pop(),Best=BestState/Modes,Mode=BestState%Modes;
            if(Closed[BestState]) continue;
            if(Best==Destination)
            {
                for(int32 I=BestState;I!=SourceState;I=Previous[I]) Route.Insert(Points[I/Modes],0);
                return true;
            }
            Closed[BestState]=true;
            for(const auto& E:Edges[Best])
            {
                const FVector A=Points[Best],B=Points[E.To]; int32 NextMode=Mode;
                if(Narrow)
                {
                    const bool East=B.X>A.X+.1 && FMath::IsNearlyEqual(A.Y,double(NarrowEastY),.1);
                    const bool West=B.X<A.X-.1 && FMath::IsNearlyEqual(A.Y,double(NarrowWestY),.1);
                    if((East && (Mode&2)) || (West && (Mode&1))) continue;
                    if(East) NextMode=1;
                    else if(West) NextMode=2;
                    else if(B.Y<NarrowWestY-.1 || B.Y>NarrowEastY+.1 || FMath::Abs(B.X-A.X)>.1) NextMode=0;
                }
                const int32 To=E.To*Modes+NextMode;
                if(Distance[BestState]+E.Cost<Distance[To])
                {
                    Distance[To]=Distance[BestState]+E.Cost; Previous[To]=BestState;
                    Push(To,Distance[To]+FVector::Dist2D(B,Goal));
                }
            }
        }
        return false;
    }
};
