#include <ros/ros.h>
#include "world.h"

using namespace std;

World::World() {
  memset(items, 0, sizeof(WorldItem*)*MAX_ITEMS);
}





