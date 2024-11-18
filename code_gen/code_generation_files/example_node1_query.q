/*
Absence of node deadlock
*/
A[] not deadlock

/*
Callback function executability
*/
E<> cb_running(0)

/*
Callback function executability
*/
E<> cb_running(1)

/*
Callback function executability
*/
E<> cb_running(2)

/*
Callback function executability
*/
E<> cb_running(3)

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(0) and cb_running(1))

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(0) and cb_running(2))

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(0) and cb_running(3))

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(1) and cb_running(2))

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(1) and cb_running(3))

/*
Mutually exclusive execution of callback functions
*/
A[] not (cb_running(2) and cb_running(3))

/*
Blocking multi-thread access to the callback group (mutual exclusion)
*/
A[] cb_running(0) imply group_closed[cb_group_table[0]]

/*
Blocking multi-thread access to the callback group (mutual exclusion)
*/
A[] cb_running(1) imply group_closed[cb_group_table[1]]

/*
Blocking multi-thread access to the callback group (mutual exclusion)
*/
A[] cb_running(2) imply group_closed[cb_group_table[2]]

/*
Blocking multi-thread access to the callback group (mutual exclusion)
*/
A[] cb_running(3) imply group_closed[cb_group_table[3]]

/*
Concurrent execution of callback functions
*/
E<> (cb_running(0) and cb_running(1))

/*
Concurrent execution of callback functions
*/
E<> (cb_running(0) and cb_running(2))

/*
Concurrent execution of callback functions
*/
E<> (cb_running(0) and cb_running(3))

/*
Concurrent execution of callback functions
*/
E<> (cb_running(1) and cb_running(2))

/*
Concurrent execution of callback functions
*/
E<> (cb_running(1) and cb_running(3))

/*
Concurrent execution of callback functions
*/
E<> (cb_running(2) and cb_running(3))

/*
Allowing multi-thread access to the callback group (reentrant)
*/
A[] not group_closed[0]

/*
Allowing multi-thread access to the callback group (reentrant)
*/
A[] not group_closed[1]

/*
Absence of callback function starvation
*/
(executor.schedule_start and executor.ready_set_copy[0] and !cb_running(0)) --> cb_running(0)

/*
Absence of callback function starvation
*/
(executor.schedule_start and executor.ready_set_copy[1] and !cb_running(1)) --> cb_running(1)

/*
Absence of callback function starvation
*/
(executor.schedule_start and executor.ready_set_copy[2] and !cb_running(2)) --> cb_running(2)

/*
Absence of callback function starvation
*/
(executor.schedule_start and executor.ready_set_copy[3] and !cb_running(3)) --> cb_running(3)
