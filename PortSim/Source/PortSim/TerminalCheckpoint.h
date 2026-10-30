#pragma once
#include "CoreMinimal.h"
#include <tuple>
#include <array>

class AQuayCrane;
class AActor;
class USceneComponent;
class UPrimitiveComponent;
class UTextRenderComponent;
class APortSiteLogistics;
class APortWorkingCrane;
class APortAGVActor;
class APortContainerActor;
class UPortEnvironmentComponent;

// In-memory checkpoints retain value copies; references are validated before
// restoration. No raw UObject serialization or reconstruction of Chaos caches.
class FTerminalCheckpoint
{
public:
    static TSharedPtr<FTerminalCheckpoint> Capture(AQuayCrane& Owner);
    bool Restore() const;
    static bool CanPredict(const AQuayCrane& Owner);
private:
    TArray<TWeakObjectPtr<UObject>> Objects;
    TArray<TFunction<void()>> Restorers;
    TArray<TFunction<void()>> SceneRestorers;
    struct FActorState { AActor* Actor; bool Hidden,Collision; };
    struct FPartState
    {
        USceneComponent* Part;
        USceneComponent* Parent;
        UPrimitiveComponent* Primitive;
        FTransform Relative;
        FName Socket;
        uint8 Collision;
        bool Visible,Hidden;
    };
    struct FLabelState { UTextRenderComponent* Label; FText Text; };
    TArray<FActorState> Actors;
    TArray<FPartState> Parts;
    TArray<FLabelState> Labels;
    template<class T> static auto Copy(const T& V) { return V; }
    template<class T,size_t N> static auto Copy(const T (&V)[N])
    {
        std::array<decltype(Copy(V[0])),N> R;
        for(size_t I=0;I<N;++I) R[I]=Copy(V[I]);
        return R;
    }
    template<class T> static void Assign(T& Target,const T& Value) { Target=Value; }
    template<class T,class U,size_t N> static void Assign(T (&Target)[N],const std::array<U,N>& Value)
    { for(size_t I=0;I<N;++I) Assign(Target[I],Value[I]); }
    template<class R,class V,size_t... I> static void RestoreTuple(R& Refs,const V& Values,std::index_sequence<I...>)
    { (Assign(std::get<I>(Refs),std::get<I>(Values)),...); }
    template<class... T> void Values(T&... Fields)
    {
        Restorers.Add([Refs=std::tie(Fields...),Saved=std::make_tuple(Copy(Fields)...)]() mutable
        { RestoreTuple(Refs,Saved,std::index_sequence_for<T...>()); });
    }
    void CaptureActor(AActor& Actor);
    void Capture(APortSiteLogistics& O);
    void Capture(APortWorkingCrane& O);
    void Capture(APortAGVActor& O);
    void Capture(APortContainerActor& O);
    void Capture(UPortEnvironmentComponent& O);
};
