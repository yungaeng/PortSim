#pragma once
#include "CoreMinimal.h"
class UWorld;
class FJsonObject;
class UStaticMeshComponent;
namespace DashboardLayout
{
    PORTSIM_API TUniquePtr<FJsonObject> Capture(UWorld* World,const FString& Run);
    PORTSIM_API TSharedRef<FJsonObject> Footprint(const FTransform& Transform,FVector MeshSizeCm);
    PORTSIM_API TSharedRef<FJsonObject> Footprint(const UStaticMeshComponent* Component);
}
