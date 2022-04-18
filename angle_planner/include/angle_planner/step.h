#include <vector>

struct iteration
{
    int prev[2];
    int next[2];
};


class step
{
  private:
    std::vector<iteration> queue;
  public:
    void put(int prev_x, int prev_y, int next_x, int next_y)
    {
        iteration new_one;
        new_one.prev[0] = prev_x;
        new_one.prev[1] = prev_y;
        new_one.next[0] = next_x;
        new_one.next[1] = next_y;
        queue.push_back(new_one);
    }
    
    void get_prev(int x, int y, int* prev)
    {
        for (int i = 0; i < queue.size(); i++)
        {
            if (queue[i].next[0] == x && queue[i].next[1] == y)
            {
                prev[0] = queue[i].prev[0];
                prev[1] = queue[i].prev[1];
                return;
            }
        }
        prev[0] = NULL;
        prev[1] = NULL;
        return;
    }
    
    void get_next(int x, int y, int* next)
    {
        for (int i = 0; i < queue.size(); i++)
        {
            if (queue[i].prev[0] == x && queue[i].prev[1] == y)
            {
                next[0] = queue[i].next[0];
                next[1] = queue[i].next[1];
                return;
            }
        }
        next[0] = NULL;
        next[1] = NULL;
        return;
    }
    
    bool where_el(int x, int y)
    {
        for (long unsigned int i = 0; i < queue.size(); i++)
        {
            if (queue[i].next[0] == x && queue[i].next[1] == y)
            {
                return true;
            }
        }
        return false;
    }
    
    bool is_empty()
    {
        if (queue.size() == 0) return true;
        else return false;
    }
    
    void update(int prev_x, int prev_y, int next_x, int next_y)
    {
        for (long unsigned int i = 0; i < queue.size(); i++)
        {
            if (queue[i].next[0] == next_x && queue[i].next[1] == next_y)
            {
                queue[i].prev[0] = prev_x;
                queue[i].prev[1] = prev_y;
            }
        }
    }
    
    void clear()
    {
        queue.clear();
    }
};
