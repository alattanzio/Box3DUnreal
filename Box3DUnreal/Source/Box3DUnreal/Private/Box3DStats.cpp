// Author: Antonio Lattanzio - emptyvessel

#include "Box3DStats.h"
#include "Box3DSubsystem.h"

DEFINE_STAT(STAT_Box3DStep);
DEFINE_STAT(STAT_Box3DPairs);
DEFINE_STAT(STAT_Box3DCollide);
DEFINE_STAT(STAT_Box3DSolve);
DEFINE_STAT(STAT_Box3DBullets);
DEFINE_STAT(STAT_Box3DSensors);

DEFINE_STAT(STAT_Box3DBodySync);
DEFINE_STAT(STAT_Box3DEvents);

DEFINE_STAT(STAT_Box3DBodies);
DEFINE_STAT(STAT_Box3DShapes);
DEFINE_STAT(STAT_Box3DContacts);
DEFINE_STAT(STAT_Box3DJoints);
DEFINE_STAT(STAT_Box3DIslands);
DEFINE_STAT(STAT_Box3DStepsPerFrame);

void UBox3DSubsystem::FBox3DFrameProfile::Accumulate(const b3Profile& P)
{
	Step += P.step;
	Pairs += P.pairs;
	Collide += P.collide;
	Solve += P.solve;
	Bullets += P.bullets;
	Sensors += P.sensors;
	++StepCount;
}

void UBox3DSubsystem::PublishStats(const FBox3DFrameProfile& Frame) const
{
#if STATS
	SET_FLOAT_STAT(STAT_Box3DStep, Frame.Step);
	SET_FLOAT_STAT(STAT_Box3DPairs, Frame.Pairs);
	SET_FLOAT_STAT(STAT_Box3DCollide, Frame.Collide);
	SET_FLOAT_STAT(STAT_Box3DSolve, Frame.Solve);
	SET_FLOAT_STAT(STAT_Box3DBullets, Frame.Bullets);
	SET_FLOAT_STAT(STAT_Box3DSensors, Frame.Sensors);
	SET_DWORD_STAT(STAT_Box3DStepsPerFrame, Frame.StepCount);

	// b3World_GetCounters walks the world, so only pay for it when the group is on.
	if (!bWorldValid || !FThreadStats::IsCollectingData(GET_STATID(STAT_Box3DBodies)))
	{
		return;
	}

	const b3Counters Counters = b3World_GetCounters(WorldId);
	SET_DWORD_STAT(STAT_Box3DBodies, Counters.bodyCount);
	SET_DWORD_STAT(STAT_Box3DShapes, Counters.shapeCount);
	SET_DWORD_STAT(STAT_Box3DContacts, Counters.contactCount);
	SET_DWORD_STAT(STAT_Box3DJoints, Counters.jointCount);
	SET_DWORD_STAT(STAT_Box3DIslands, Counters.islandCount);
#endif
}
