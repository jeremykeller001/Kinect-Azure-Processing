#pragma once

#include "Ply.h"

#ifndef PCL_UTILS_H
#define PCL_UTILS_H

class PclUtils
{
public:
	void applyNearestNeighborFilter(Ply pointCloud);
};
#endif
