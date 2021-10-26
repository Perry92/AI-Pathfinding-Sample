// Fergus Marsden 2020.


#include "FJMDistancePathFinder.h"
#include <Kismet/GameplayStatics.h>
#include "NavigationSystem.h"
#include "NavigationData.h"
#include "WorldWarTwo/Library/StaticFunctions.h"
#include "WorldWarTwo/NavData/FJMAIDistancePathNode.h"
#include "NavMesh/RecastNavMesh.h"
#include "DrawDebugHelpers.h"
#include <Components/BoxComponent.h>

// Sets default values
AFJMDistancePathFinder::AFJMDistancePathFinder()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	ThisRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ThisRoot"));
	SetRootComponent(ThisRoot);

	NavArea = CreateDefaultSubobject<UBoxComponent>(TEXT("NavArea"));
	NavArea->SetupAttachment(ThisRoot);
	NavArea->SetBoxExtent(FVector(400.f, 400.f, 200.f));
	NavArea->SetRelativeLocation(FVector(0.f, 0.f, 200.f));

	NavArea->SetCollisionObjectType(ECC_WorldDynamic);
	NavArea->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore);
}

void AFJMDistancePathFinder::CreatePathNodes()
{
	//Find Recast Navmesh
	TArray<AActor*> FoundNavs;

	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ARecastNavMesh::StaticClass(), FoundNavs);

	if (FoundNavs.Num() > 0) { NavData = Cast<ARecastNavMesh>(FoundNavs[0]); }

	if (NavData)
	{
		CreatedPathNodes.Empty();
		FVector NavBounds = NavArea->GetScaledBoxExtent();
		float AreaWidth = NavBounds.X * 2;
		float AreaLength = NavBounds.Y * 2;

		int32 WidthNodesNum = FMath::TruncToInt((AreaWidth / NodeSpacing));
		int32 LengthNodesNum = FMath::TruncToInt((AreaLength / NodeSpacing));

		FString NameBase = "PathNode_";
		for (int32 Index = 0; Index < LengthNodesNum; Index++)
		{
			int32 Spacing = Index * NodeSpacing;
			FVector RightSpacing = GetActorRightVector() * Spacing;
			FVector BackLeftCorner = (GetActorRightVector() * (AreaWidth * -0.5)) + (GetActorForwardVector() * (AreaLength * -0.5));

			FVector LengthNodeVector = GetActorLocation() + (RightSpacing + BackLeftCorner);
			FVector GroundLocation = FindNodeLocationOnTerrain(LengthNodeVector);

			if (IsNavigationAtVector(GroundLocation))
			{
				FString NameX = FString::FromInt(Spacing);
				FString NameY = FString::FromInt(0);
				FString LName = NameBase;
				LName.Append(NameX);
				LName.Append(NameY);

				FAIPathNode LengthNode = FAIPathNode();
				LengthNode.NodeLocation = GroundLocation + FVector(0,0,50);
				LengthNode.HasNavData = true;

				CreatedPathNodes.AddUnique(LengthNode);
			}
			///*
			for (int32 WidthIndex = 1; WidthIndex < WidthNodesNum; WidthIndex++)
			{
				int32 WidthSpacing = WidthIndex * NodeSpacing;
				FVector FwdSpacing = GetActorForwardVector() * WidthSpacing;

				FVector WidthNodeVector = LengthNodeVector + FwdSpacing;
				FVector WidthGroundLocation = FindNodeLocationOnTerrain(WidthNodeVector);

				if (IsNavigationAtVector(WidthGroundLocation))
				{
					FString WNameX = FString::FromInt(Spacing);
					FString WNameY = FString::FromInt(WidthSpacing);
					FString WName = NameBase;
					WName.Append(WNameX);
					WName.Append(WNameY);

					FAIPathNode WidthNode = FAIPathNode();
					WidthNode.NodeLocation = WidthGroundLocation + FVector(0, 0, 50);
					WidthNode.HasNavData = true;

					CreatedPathNodes.AddUnique(WidthNode);
				}
			}
		}
	}
}

FVector AFJMDistancePathFinder::FindNodeLocationOnTerrain(FVector OriginVector)
{
	FVector GroundVector;
	FVector TestVector = OriginVector.Y == 0.f ? FVector(OriginVector.X, 1.f, OriginVector.Z) : OriginVector;
	if (UStaticFunctions::GetGroundLocationAtVector(this, TestVector, GroundVector))
	{
		return GroundVector;
	}

	return OriginVector;
}

bool AFJMDistancePathFinder::IsNavigationAtVector(FVector OriginVector)
{
	AFJMBaseBuilding* DudBuilding;
	if (UStaticFunctions::VectorIsInBuilding(this, OriginVector, DudBuilding))
	{
		return false;
	}
	else if (NavData)
	{
		FNavLocation OUTLocation;
		bool Navigable = UNavigationSystemV1::GetNavigationSystem(this)->ProjectPointToNavigation(OriginVector, OUTLocation, NavQueryExtent, NavData);

		return Navigable;
	}

	return false;
}

void AFJMDistancePathFinder::CalculateAllNeighbours()
{
}

TArray<FAIPathNode*> AFJMDistancePathFinder::GetNodeNeighbours(FAIPathNode* OriginNode)
{
	TArray<FAIPathNode*> ReturnArray;
	FVector NodeLocation = OriginNode->NodeLocation;
	for (int32 NodeIndex = 0; NodeIndex < CreatedPathNodes.Num(); NodeIndex++)
	{
		FAIPathNode* OtherNode = &CreatedPathNodes[NodeIndex];
		if (NodeLocation != OtherNode->NodeLocation)
		{
			if ((NodeLocation - OtherNode->NodeLocation).Size() < (NodeSpacing * 1.65)) { ReturnArray.Add(OtherNode); }
			if (OriginNode->Neighbours.Num() > 8) { break; }
		}
	}
	
	return ReturnArray;
}
///*
FAIPathNode* AFJMDistancePathFinder::GetNodeAtLocation(FVector Location)
{
	for (FAIPathNode& Node : CreatedPathNodes)
	{
		if ((Location - Node.NodeLocation).Size() < (NodeSpacing * 1.75)) { return &Node; }
	}

	return nullptr;
}

TArray<FAIPathNode> AFJMDistancePathFinder::FindDistancePath(FVector Home, FVector Goal)
{
	FAIPathNode* StartNode = GetNodeAtLocation(Home);
	FAIPathNode* EndNode = GetNodeAtLocation(Goal);

	if (StartNode == nullptr) { return TArray<FAIPathNode>(); }
	if (EndNode == nullptr) { return TArray<FAIPathNode>(); }

	TArray<FAIPathNode*> OpenSet;
	TArray<FAIPathNode*> ClosedSet;

	OpenSet.AddUnique(StartNode);

	while (OpenSet.Num() > 0)
	{
		OpenSet.HeapSort([](const FAIPathNode& A, const FAIPathNode& B) {
			return A.GetCurrentFCost() < B.GetCurrentFCost();
		});

		FAIPathNode* CurrentNode = OpenSet[0];
		if (OpenSet.Num() > 1)
		{
			FAIPathNode* EvalNode = OpenSet[1];
			bool HasLowerFCost = EvalNode->GetCurrentFCost() < CurrentNode->GetCurrentFCost();
			bool HasBetterHCost = EvalNode->GetCurrentFCost() == CurrentNode->GetCurrentFCost() && EvalNode->GetCurrentHCost() < CurrentNode->GetCurrentHCost();
			if (HasLowerFCost || HasBetterHCost)
			{
				CurrentNode = OpenSet[1];
			}
		}

		OpenSet.Remove(CurrentNode);
		ClosedSet.AddUnique(CurrentNode);

		if (CurrentNode == EndNode) {return RetracePath(StartNode, EndNode); break; }

		for (FAIPathNode* NeighbourNode : GetNodeNeighbours(CurrentNode))
		{
			if (ClosedSet.Contains(NeighbourNode)) { continue; }

			int32 NewCostToNeighbour = CurrentNode->GetCurrentGCost() + abs(CurrentNode->CalculateCostToOtherNode(NeighbourNode));
			if (NewCostToNeighbour < NeighbourNode->GetCurrentGCost() || !OpenSet.Contains(NeighbourNode))
			{
				NeighbourNode->SetCurrentGCost(NewCostToNeighbour);
				NeighbourNode->SetCurrentHCost(abs(NeighbourNode->CalculateCostToOtherNode(EndNode)));
				NeighbourNode->UpdateCurrentFCost();

				NeighbourNode->SetParentNode(CurrentNode);

				if (!OpenSet.Contains(NeighbourNode)) { OpenSet.AddUnique(NeighbourNode); }
			}
		}
	}

	return TArray<FAIPathNode>();
}

TArray<FAIPathNode> AFJMDistancePathFinder::RetracePath(FAIPathNode* StartNode, FAIPathNode* EndNode)
{
	TArray<FAIPathNode> DistancePath;
	FAIPathNode* CurrentNode = EndNode;
	while (CurrentNode != StartNode)
	{
		DistancePath.AddUnique(*CurrentNode);
		CurrentNode = CurrentNode->GetParentNode();
	}

	Algo::Reverse(DistancePath);

	for (FAIPathNode Node : DistancePath)
	{
		if (Debug)
		{
			UE_LOG(LogTemp, Warning, TEXT("ASTARPATHING: Path Point found at: %f, %f, %f"), Node.NodeLocation.X, Node.NodeLocation.Y, Node.NodeLocation.Z);
			DrawDebugSphere(GetWorld(), Node.NodeLocation, 35.f, 12, FColor::Blue, false, 5.f);
		}
	}

	return DistancePath;
}
//*/
// Called when the game starts or when spawned
void AFJMDistancePathFinder::BeginPlay()
{
	Super::BeginPlay();
}

