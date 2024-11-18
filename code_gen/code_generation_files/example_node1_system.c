// System name: example_node1

// Callbacks
TMR1 = Callback(0);
SUB1 = Callback(1);
SRV1 = Callback(2);
CLT1 = Callback(3);

// Callback group
RGroup1 = CallbackGroup(REENTRANT, 0);
MEGroup1 = CallbackGroup(EXCLUSIVE, 1);

// Executor and Threads
executor = Executor();
thread0 = Thread(0);
thread1 = Thread(1);
thread2 = Thread(2);
thread3 = Thread(3);

// Initialize the system
system
executor,
thread0,
thread1,
thread2,
thread3,
RGroup1,
MEGroup1,
TMR1,
SUB1,
SRV1,
CLT1;