#include "communication.h"

volatile atomic<bool> Communication::is_exit_thread{false};
volatile atomic<bool> Communication::is_halt_process{false};

Communication::Communication() {
    this->sign_queue = new boost::lockfree::queue<sign_item_t, boost::lockfree::fixed_sized<true>>(256);
    sign_command = CMD_NONE;
    sign_distance = 1e9;
}
void Communication::halt_process() {
    is_halt_process = true;
}
void Communication::exit_process() {
    is_exit_thread = true;
}

void Communication::continue_process() {
    is_halt_process = false;
}
