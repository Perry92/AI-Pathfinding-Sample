// Fergus Marsden 2020.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WorldWarTwo/Library/FJMStructs.h"
#include "FJMDistancePathFinder.generated.h"

class ARecastNavMesh;
class USceneComponent;
class UBoxComponent;
class UFJMAIDistancePathNode;

struct FAIPathNode;

UCLASS()
class WORLDWARTWO_API AFJMDistancePathFinder : public AActor
{
	GENERATED_BODY()

////////////////////////////////
// Components /////////////////
//////////////////////////////

	UPROPERTY(Category = Commander, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	USceneComponent* ThisRoot = nullptr;

	UPROPERTY(Category = Commander, VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	UBoxComponent* NavArea = nullptr;

public:	
	// Sets default values for this actor's properties
	AFJMDistancePathFinder();

////////////////////////////////
// Node Creation //////////////
//////////////////////////////
public:
	/* This function uses the current navigation system to place nodes on areas where navmesh has been created.
	* it will check the area for navmesh and ensure that no obstacles will obstruct movement before placing.*/
	UFUNCTION(BlueprintCallable)
	void CreatePathNodes();

protected:
	/*How far apart the nodes should be.
	*Ideally we need to have this as high as possible to reduce any problems with performance
	*However on maps where narrow tracks exist it is better to have a lower setting to ensure a path can be found*/
	UPROPERTY(EditAnywhere)
	int32 NodeSpacing = 250;

	//An array used to store all created path nodes.
	UPROPERTY()
	TArray<FAIPathNode> CreatedPathNodes;

	//The distance at which there must be navmesh in order for a node to be created.
	//unusual erros occur when we reduce the z Axis.
	UPROPERTY(EditAnywhere)
	FVector NavQueryExtent = FVector(50, 50, 5000);

	FVector FindNodeLocationOnTerrain(FVector OriginVector);

	bool IsNavigationAtVector(FVector OriginVector);

	//Depreciated and no longer used - see GetNodeNeighbours.
	void CalculateAllNeighbours();

	//Gets all the neighbouring nodes of the given node - used when we check for path.
	TArray<FAIPathNode*> GetNodeNeighbours(FAIPathNode* OriginNode);

	//the navmesh data that is placed in the current level.
	ARecastNavMesh* NavData = nullptr;

////////////////////////////////
// Path Finder ////////////////
//////////////////////////////

public:
	/*Find the shortest path between two nodes using A-star pathfinding*/
	UFUNCTION(BlueprintCallable)
	TArray<FAIPathNode> FindDistancePath(FVector Home, FVector Goal);

	//traces the path back once the end node has been found and returns the path we can pass to a character.
	TArray<FAIPathNode> RetracePath(FAIPathNode* StartNode, FAIPathNode* EndNode);

protected:
	FAIPathNode* GetNodeAtLocation(FVector Location);

////////////////////////////////
// General ////////////////////
//////////////////////////////
public:
	UFUNCTION(BlueprintCallable)
	TArray<FAIPathNode> GetAllPathNodes() const
	{
		return CreatedPathNodes;
	}

	UFUNCTION(BlueprintCallable)
	bool GetDebug() const
	{
		return Debug;
	}

	int32 GetNodeSpacing() const
	{
		return NodeSpacing;
	}

	TArray<FAIPathNode>& GetPathNodes()
	{
		return CreatedPathNodes;
	}

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere)
	bool Debug = false;

};
