#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__


// microROS system state
typedef enum _SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} SystemState;



#endif