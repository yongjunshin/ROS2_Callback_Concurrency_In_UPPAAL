// User parameters
const int NUM_CB = 4;    // user config param
const int NUM_CB_GROUP = 2;    // user config param
const int NUM_THREAD = 4;    // user config param

const int MAX_CYCLE = 1;

typedef int[0, NUM_CB-1] CbId;
typedef int[0, NUM_CB_GROUP-1] GroupId;
typedef int[0, NUM_THREAD-1] ThreadId;

typedef int[0, 3] CbType;
const CbType TMR = 0;
const CbType SUB = 1;
const CbType SRV = 2;
const CbType CLT = 3;

typedef int[0, 1] GroupType;
const GroupType REENTRANT = 0;
const GroupType EXCLUSIVE = 1;

const GroupId cb_group_table[NUM_CB] = {0, 0, 1, 1};    // user config param
const CbType cb_type_table[NUM_CB] = {TMR, SUB, SRV, CLT};    // user config param

// Callback group
bool group_closed[NUM_CB_GROUP];
chan group_close[NUM_CB_GROUP];
chan group_open[NUM_CB_GROUP];

// Callback
bool ready_set[NUM_CB];

// Thread
chan cb_assign_signal[NUM_THREAD];
broadcast chan thread_free;
// int[0, NUM_THREAD] num_running_thread;

bool thread_running[NUM_THREAD];
CbId thread_cb_assign_table[NUM_THREAD];

bool cb_running(CbId cb_id){
    int t_id;
    for (t_id = 0; t_id < NUM_THREAD; t_id++){
        if (thread_running[t_id]){
            if (thread_cb_assign_table[t_id] == cb_id){
                return true;
            }
        }
    }
    return false;
}

CbId get_thread_running_cb(ThreadId t_id){
    return thread_cb_assign_table[t_id];
}

GroupId get_thread_running_group(ThreadId t_id){
    return cb_group_table[thread_cb_assign_table[t_id]];
}