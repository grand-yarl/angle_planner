#include <vector>

struct point
{
    int coord[2];
    double cost;
};


class que
{
  private:
    std::vector<point> queue;
  public:
    void put(int x, int y, double cost)
    {
        point new_one;
        new_one.coord[0] = x;
        new_one.coord[1] = y;
        new_one.cost = cost;
        int index = 0;
        if (queue.empty())
        {
            queue.insert(queue.begin() + index, new_one);
            return;
        }
        while (new_one.cost > queue[index].cost && index < queue.size() - 1)
        {
            index++;
        }
        queue.insert(queue.begin() + index, new_one);
    }
    
    point pop_min()
    {
        point output = queue[0];
        queue.erase(queue.begin());
        return output;
    }
    
    point pop_last()
    {
        point output = queue[queue.size() - 1];
        queue.erase(queue.begin() + queue.size() - 1);
        return output;
    }
    
    double take_cost(int x, int y)
    {
        for (int i = 0; i < queue.size(); i++)
        {
            if (queue[i].coord[0] == x && queue[i].coord[1] == y)
            {
                return queue[i].cost;
            }
        }
        return -1;
    }
    
    point take_point(double cost)
    {
        double x, y;
        point output;
        for (long unsigned int i = 0; i < queue.size(); i++)
        {
            if (queue[i].cost == cost)
            {
                output = queue[i];
                return output;
            }
        }
        output.coord[0] = NULL;
        output.coord[1] = NULL;
        output.cost = NULL;
        return output;
    }
    
    bool where_el(int x, int y)
    {
        for (long unsigned int i = 0; i < queue.size(); i++)
        {
            if (queue[i].coord[0] == x && queue[i].coord[1] == y)
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
    
    void update(int x, int y, double cost)
    {
        for (long unsigned int i = 0; i < queue.size(); i++)
        {
            if (queue[i].coord[0] == x && queue[i].coord[1] == y)
            {
                queue[i].cost = cost;
            }
        }
    }
};
