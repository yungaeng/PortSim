#include "ShipStowagePlanning.h"
#include "TerminalLayout.h"
#include "Misc/FileHelper.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"

bool FPortShipStowageAssumptions::Load(const FString& Path,FString& Error)
{
    FString Text; TSharedPtr<FJsonObject> Root;
    if(!FFileHelper::LoadFileToString(Text,*Path) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Root) || !Root.IsValid())
    { Error=TEXT("Missing/invalid Stowage_Planning.json"); return false; }
    if(!Root->TryGetStringField(TEXT("basis"),Basis) || Basis!=TEXT("simulation_assumptions_not_vessel_approved"))
    { Error=TEXT("Ship planning assumptions must declare their simulation basis"); return false; }
    auto Read=[&](const TCHAR* Key,double& Value)
    {
        return Root->TryGetNumberField(Key,Value) && FMath::IsFinite(Value) && Value>=0;
    };
    if(!Read(TEXT("lightship_kg"),LightshipKg) || !Read(TEXT("lightship_kg_m"),LightshipKGM) ||
        !Read(TEXT("km_m"),KMM) || !Read(TEXT("free_surface_moment_kg_m"),FreeSurfaceMomentKgM) ||
        !Read(TEXT("minimum_gm_m"),MinimumGMM) || !Read(TEXT("maximum_gm_m"),MaximumGMM) ||
        !Read(TEXT("maximum_heel_deg"),MaximumHeelDegrees) || !Read(TEXT("maximum_stack_kg"),MaximumStackKg) ||
        !Read(TEXT("maximum_lashing_n"),MaximumLashingN) || !Read(TEXT("lateral_acceleration_mps2"),LateralAccelerationMps2) ||
        !Read(TEXT("deck_height_m"),DeckHeightM) || LightshipKg<=0 || KMM<=LightshipKGM || MinimumGMM<=0 ||
        MaximumGMM<=MinimumGMM || MaximumStackKg<=0 || MaximumLashingN<=0 || MaximumHeelDegrees<=0 ||
        LateralAccelerationMps2<=0)
    { Error=TEXT("Invalid ship planning numeric assumptions"); return false; }
    SnapshotJson=Text;
    // Optional geometry keys preserve older configuration files.
    auto Optional=[&](const TCHAR* Key,double& Value) { return !Root->HasField(Key) || Read(Key,Value); };
    if(!Optional(TEXT("bay_pitch_m"),BayPitchM) || !Optional(TEXT("row_pitch_m"),RowPitchM) ||
        !Optional(TEXT("container_height_m"),ContainerHeightM) || !Optional(TEXT("twin20_pitch_m"),Twin20PitchM) ||
        !Optional(TEXT("maximum_longitudinal_cg_m"),MaximumLongitudinalCGM))
    { Error=TEXT("Invalid arrival cell geometry"); return false; }
    if(Root->HasField(TEXT("use_deck_zones")) && !Root->TryGetBoolField(TEXT("use_deck_zones"),UseDeckZones))
    { Error=TEXT("Invalid arrival deck zone flag"); return false; }
    return true;
}

FPortShipStowageResult PlanShipStowage(const TArray<FPortShipPlanningCargo>& Cargo,
    const FPortShipStowageAssumptions& A,bool bBalanceDischarge)
{
    FPortShipStowageResult Result;
    for(double V:{A.MaximumStackKg,A.MaximumLashingN,A.MaximumHeelDegrees,A.DeckHeightM})
        if(!FMath::IsFinite(V) || V<=0) { Result.Reason=TEXT("Invalid ship limit"); return Result; }
    if(!FMath::IsFinite(A.LightshipKg) || A.LightshipKg<=0 || A.Bays<=0 || A.Rows<=0 || A.Tiers<=0 ||
        A.Bays>100 || A.Rows>30 || A.Tiers>20 || !FMath::IsFinite(A.KMM) || !FMath::IsFinite(A.LightshipKGM) ||
        !FMath::IsFinite(A.FreeSurfaceMomentKgM) || A.FreeSurfaceMomentKgM<0 ||
        !FMath::IsFinite(A.MinimumGMM) || !FMath::IsFinite(A.MaximumGMM) || A.MinimumGMM<=0 ||
        A.MaximumGMM<=A.MinimumGMM || A.MaximumStackKg<=0 || A.MaximumLashingN<=0 ||
        !FMath::IsFinite(A.LateralAccelerationMps2) || A.LateralAccelerationMps2<=0)
    { Result.Reason=TEXT("Invalid ship stability/stack/lashing inputs"); return Result; }
    if(!FMath::IsFinite(A.BayPitchM) || A.BayPitchM<13.716 || !FMath::IsFinite(A.RowPitchM) || A.RowPitchM<2.438 ||
        !FMath::IsFinite(A.ContainerHeightM) || A.ContainerHeightM<2.591 || !FMath::IsFinite(A.Twin20PitchM) ||
        A.Twin20PitchM<6.058 || A.Twin20PitchM+6.058>A.BayPitchM ||
        !FMath::IsFinite(A.MaximumLongitudinalCGM) || A.MaximumLongitudinalCGM<=0)
    { Result.Reason=TEXT("Invalid or overlapping ship cell geometry"); return Result; }
    struct FColumn { TArray<int32> CargoIndices; double Mass=0, Moment=0; };
    // One physical cell accepts either one long stack or two independent 20ft
    // stacks. No long box is supported by one short box or an empty pad.
    TArray<FColumn> Columns; Columns.SetNum(A.Bays*A.Rows*2);
    TArray<int32> CellLength; CellLength.Init(0,A.Bays*A.Rows);
    TArray<int32> Order; TSet<int32> IDs;
    for(int32 I=0;I<Cargo.Num();++I)
    {
        if(IDs.Contains(Cargo[I].ID) || !Cargo[I].Information.Valid() || Cargo[I].CoGOffsetM.ContainsNaN() ||
            FMath::Abs(Cargo[I].CoGOffsetM.Y)>1.219 || FMath::Abs(Cargo[I].CoGOffsetM.Z)>1.2955 || !FMath::IsFinite(Cargo[I].GrossKg) ||
            Cargo[I].GrossKg<=0 || (Cargo[I].LengthFt!=20 && Cargo[I].LengthFt!=40 && Cargo[I].LengthFt!=45))
        { Result.Reason=TEXT("Invalid/duplicate ship booking"); return Result; }
        IDs.Add(Cargo[I].ID); Order.Add(I);
        if(FMath::Abs(Cargo[I].CoGOffsetM.X)>(Cargo[I].LengthFt==20?3.029:Cargo[I].LengthFt==40?6.096:6.858))
        { Result.Reason=TEXT("Longitudinal container CoG is outside its envelope"); return Result; }
    }
    // Later port calls load below earlier calls. Within that group, heavy first.
    Order.StableSort([&](int32 X,int32 Y)
    {
        if(Cargo[X].Information.PortCallOrder!=Cargo[Y].Information.PortCallOrder)
            return Cargo[X].Information.PortCallOrder>Cargo[Y].Information.PortCallOrder;
        if(Cargo[X].GrossKg!=Cargo[Y].GrossKg) return Cargo[X].GrossKg>Cargo[Y].GrossKg;
        return Cargo[X].ID<Cargo[Y].ID;
    });
    double Displacement=A.LightshipKg, VerticalMoment=A.LightshipKg*A.LightshipKGM, TransverseMoment=0, LongitudinalMoment=0;
    Result.CorrectedGMM=A.KMM-A.LightshipKGM-A.FreeSurfaceMomentKgM/A.LightshipKg;
    double WorkByZone[3]={};
    auto DischargeWork=[](const FVector& Position,int32 Zone)
    {
        // Estimate one STS cycle and add the extra gantry travel away from its pad.
        return 120.+2.*FMath::Abs(Position.X-(Zone-1)*TerminalLayout::STSSpacingM);
    };
    for(int32 Index:Order)
    {
        const auto& C=Cargo[Index];
        int32 Best=INDEX_NONE; double BestScore=TNumericLimits<double>::Max(),BestGM=0,BestHeel=0,BestLash=0;
        FVector BestPosition=FVector::ZeroVector;
        for(int32 Col=0;Col<Columns.Num();++Col)
        {
            const int32 Cell=Col/2, Pad=Col%2, Bay=Cell/A.Rows, Row=Cell%A.Rows;
            if((C.LengthFt!=20 && Pad!=0) || (CellLength[Cell]!=0 && CellLength[Cell]!=C.LengthFt)) continue;
            if(A.UseDeckZones)
            {
                // Concept-vessel fixtures: powered centre rows; separated DG
                // end bays; 45ft-capable end cells. These are NOT an IMDG table.
                const bool EndBay=A.IsEndBay(Bay);
                const bool Powered=A.IsPoweredRow(Row);
                if(C.LengthFt==45 && !EndBay) continue;
                if(C.Information.HandlingType==TEXT("reefer")) { if(!Powered) continue; }
                else if(Powered && !EndBay) continue;
                if(C.Information.HazardGroup==TEXT("flammable") && !A.IsFlammableBay(Bay)) continue;
                if(C.Information.HazardGroup==TEXT("corrosive") && !A.IsCorrosiveBay(Bay)) continue;
                if(C.Information.HazardGroup==TEXT("none") && EndBay && C.LengthFt!=45) continue;
            }
            const auto& Stack=Columns[Col]; const int32 Tier=Stack.CargoIndices.Num();
            if(Tier>=A.Tiers || Stack.Mass+C.GrossKg>A.MaximumStackKg) continue;
            if(C.LengthFt==20 && Stack.Mass+Columns[Col^1].Mass+C.GrossKg>A.MaximumStackKg) continue;
            if(Tier>0)
            {
                const auto& Lower=Cargo[Stack.CargoIndices.Last()];
                if(Lower.LengthFt!=C.LengthFt || Lower.Information.PortCallOrder<C.Information.PortCallOrder ||
                    Lower.Information.HazardGroup!=C.Information.HazardGroup ||
                    Lower.Information.HandlingType!=C.Information.HandlingType || Lower.GrossKg<C.GrossKg) continue;
            }
            bool Supported=true; double Above=C.GrossKg;
            for(int32 N=Tier-1;N>=0;--N)
            {
                const auto& Lower=Cargo[Stack.CargoIndices[N]];
                if(Above>Lower.Information.PermittedSuperimposedKg) { Supported=false; break; }
                Above+=Lower.GrossKg;
            }
            if(!Supported) continue;
            const FVector Position((Bay-(A.Bays-1)*.5)*A.BayPitchM+(C.LengthFt==20?(Pad?1.:-1.)*A.Twin20PitchM*.5:0.),
                (Row-(A.Rows-1)*.5)*A.RowPitchM,A.DeckHeightM+(Tier+.5)*A.ContainerHeightM);
            const double X=Position.X+C.CoGOffsetM.X,Y=Position.Y+C.CoGOffsetM.Y,Z=Position.Z+C.CoGOffsetM.Z;
            const double Delta=Displacement+C.GrossKg;
            const double LCG=(LongitudinalMoment+C.GrossKg*X)/Delta;
            if(FMath::Abs(LCG)>A.MaximumLongitudinalCGM) continue;
            const double GM=A.KMM-(VerticalMoment+C.GrossKg*Z)/Delta-A.FreeSurfaceMomentKgM/Delta;
            if(!FMath::IsFinite(GM) || GM<A.MinimumGMM || GM>A.MaximumGMM) continue;
            const double Heel=FMath::RadiansToDegrees(FMath::Atan2(FMath::Abs(TransverseMoment+C.GrossKg*Y)/Delta,GM));
            if(Heel>A.MaximumHeelDegrees) continue;
            // Conservative static overturning-moment estimate shared by four
            // lashings. It is a simulation proxy, not vessel-approved lashing software.
            const double Moment=Stack.Moment+C.GrossKg*A.LateralAccelerationMps2*(Z-A.DeckHeightM);
            const double Lashing=Moment/(4*2.438);
            if(Lashing>A.MaximumLashingN) continue;
            // Preserve unused columns for later port groups whose heavier boxes
            // cannot sit above today's lighter stack. Reuse compatible stacks
            // after all safety limits pass, rather than exhausting the floor first.
            double Score=Heel*1000.+FMath::Abs(LCG)*100.+(Tier==0?500.:Tier*10.)+
                (C.LengthFt==20 && !Columns[Col^1].CargoIndices.IsEmpty()?-100.:0.);
            if(bBalanceDischarge)
            {
                const int32 Zone=TerminalLayout::VesselDischargeZone(Position.X);
                const double Work=DischargeWork(Position,Zone);
                // Marginal growth in the squared STS workloads after hard ship checks pass.
                Score+=.002*(2.*WorkByZone[Zone]+Work)*Work;
            }
            if(Score<BestScore) { Best=Col; BestScore=Score; BestGM=GM; BestHeel=Heel; BestLash=Lashing; BestPosition=Position; }
        }
        if(Best==INDEX_NONE) { Result.Reason=TEXT("No placement satisfies port access, GM, heel, stack and lashing limits"); return Result; }
        auto& Stack=Columns[Best]; const int32 Tier=Stack.CargoIndices.Num();
        const double Y=BestPosition.Y+C.CoGOffsetM.Y,Z=BestPosition.Z+C.CoGOffsetM.Z;
        LongitudinalMoment+=C.GrossKg*(BestPosition.X+C.CoGOffsetM.X);
        Displacement+=C.GrossKg; VerticalMoment+=C.GrossKg*Z; TransverseMoment+=C.GrossKg*Y;
        Stack.Mass+=C.GrossKg; Stack.Moment+=C.GrossKg*A.LateralAccelerationMps2*(Z-A.DeckHeightM);
        FPortShipPlacement Placement{C.ID,Best/2/A.Rows+1,(Best/2)%A.Rows+1,Tier+1,BestGM,BestHeel,Stack.Mass,BestLash};
        Placement.Subslot=C.LengthFt==20?Best%2+1:0;
        Placement.BelowCargoID=Stack.CargoIndices.IsEmpty()?INDEX_NONE:Cargo[Stack.CargoIndices.Last()].ID;
        Placement.PositionM=BestPosition;
        Stack.CargoIndices.Add(Index); CellLength[Best/2]=C.LengthFt;
        Result.Placements.Add(Placement);
        Result.CorrectedGMM=BestGM; Result.HeelDegrees=BestHeel;
        Result.LongitudinalCGM=LongitudinalMoment/Displacement;
        if(bBalanceDischarge)
        {
            const int32 Zone=TerminalLayout::VesselDischargeZone(BestPosition.X);
            WorkByZone[Zone]+=DischargeWork(BestPosition,Zone);
        }
    }
    Result.Feasible=true; Result.Reason=TEXT("Feasible under explicit simulation assumptions");
    return Result;
}
