#pragma once
#include "CoreMinimal.h"

// Directed, axis-aligned road centre lines. Endpoints and crossings are split
// into graph nodes; endpoints off the road never get a straight-line shortcut.
class FAGVRoadNetwork
{
public:
    struct FSegment { FVector A,B; bool Both; };
    TArray<FSegment> Roads;
    void Add(FVector A,FVector B,bool Both=false)
    {
        if(!A.Equals(B,.1) && (FMath::IsNearlyEqual(A.X,B.X,.1) || FMath::IsNearlyEqual(A.Y,B.Y,.1)))
            Roads.Add({A,B,Both});
    }
    static bool Contains(const FSegment& S,FVector P)
    {
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
        TArray<double> Distance; Distance.Init(TNumericLimits<double>::Max(),Points.Num());
        TArray<int32> Previous; Previous.Init(INDEX_NONE,Points.Num());
        TArray<bool> Closed; Closed.Init(false,Points.Num()); Distance[Source]=0;
        for(int32 Count=0;Count<Points.Num();++Count)
        {
            int32 Best=INDEX_NONE; double Score=TNumericLimits<double>::Max();
            for(int32 I=0;I<Points.Num();++I) if(!Closed[I] && Distance[I]<TNumericLimits<double>::Max())
            {
                const double F=Distance[I]+FVector::Dist2D(Points[I],Goal);
                if(F<Score) { Score=F; Best=I; }
            }
            if(Best==INDEX_NONE) return false;
            if(Best==Destination)
            {
                for(int32 I=Destination;I!=Source;I=Previous[I]) Route.Insert(Points[I],0);
                return true;
            }
            Closed[Best]=true;
            for(const auto& E:Edges[Best]) if(Distance[Best]+E.Cost<Distance[E.To])
            { Distance[E.To]=Distance[Best]+E.Cost; Previous[E.To]=Best; }
        }
        return false;
    }
};
