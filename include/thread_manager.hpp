#pragma once
#include <thread>
#include <vector>

class thread_manager
{
private:
    std::vector<std::jthread> threads;
    template<typename Head, typename... Args>
    bool check_valid(Head&& head, Args&&... args)
    {
        return head != nullptr;
    }
public:
    template<typename F,typename... Args>
    void emplace_back(F&& f, Args&&... args)
    {
        if (check_valid(std::forward<Args>(args)...))
            threads.emplace_back(std::forward<F>(f), std::forward<Args>(args)...);
    }

    void join()
    {
        threads.clear();
    }
};
