#ifndef PICKUP_STATES_H
#define PICKUP_STATES_H

namespace pickup_states 
{
  enum class MasterPickupStates {
    OFF,
    SEARCH,
    ACTION
  };

  enum class GlobalPickupStates {
    OFF,
    APPROACH,
    SEARCH,
    ATTEMPT_PICKUP, 
    DROPOFF
  };

  enum class LocalPickupState {
    OFF,
    BRICK_ALIGNMENT,
    DESCENT,
    TOUCHDOWN_ALIGNMENT,
    TOUCHDOWN
  };

}

#endif /* PICKUP_STATES_H */