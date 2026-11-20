#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

#define K_THREAD_A_SLEEP_MS    100U
#define K_THREAD_B_SLEEP_MS    300U

#define K_THREAD_A_STACK_SIZE  1024U
#define K_THREAD_B_STACK_SIZE  1024U
#define K_PRIORITY_THREAD_A    1 
#define K_PRIORITY_THREAD_B    1 

void thread_a_fn(void* p1, void* p2, void* p3)
{
    LOG_INF("[A] Thread A launched!!");
    while (1) {
        LOG_INF("[A] is running tick=%u", k_uptime_get_32());
        k_msleep(K_THREAD_A_SLEEP_MS);
    }
}

void thread_b_fn(void* p1, void* p2, void* p3)
{
    LOG_INF("[B] Thread B launched!!");
    while (1) {
        LOG_INF("[B] is running tick=%u", k_uptime_get_32());
        k_msleep(K_THREAD_B_SLEEP_MS);
    }
}

K_THREAD_DEFINE(thread_a, K_THREAD_A_STACK_SIZE, thread_a_fn, NULL, NULL, NULL, K_PRIORITY_THREAD_A, 0, 0);
K_THREAD_DEFINE(thread_b, K_THREAD_A_STACK_SIZE, thread_b_fn, NULL, NULL, NULL, K_PRIORITY_THREAD_B, 0, 0);

int main()
{
    LOG_INF("Application started!!");
    return 0;
}