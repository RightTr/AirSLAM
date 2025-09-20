#ifndef QUEUE_H_
#define QUEUE_H_

#include <iostream>

#define MAXCAPACITY 10

template <class T> class Queue
{
    private:
        T *data;
        int head, tail;
        int capacity;
        int size;

    public:
        Queue(int capacity):capacity(capacity), size(0), head(0), tail(0)
        {
            data = new T[capacity];
        }


        Queue():capacity(MAXCAPACITY), size(0), head(0), tail(0)
        {
            data = new T[capacity];
        }

        ~Queue()
        {
            delete[] data;
        }


        bool IsEmpty() const
        {
            if(size == 0) return true;
            else return false;
        }


        bool IsFull() const
        {
            if(size == capacity) return true;
            else return false;
        }
        
        bool Push(const T &value)
        {
            if(IsFull()) 
            {
                throw("Queue is Full!");
                return false;
            }

            data[tail] = value;
            tail = (tail + 1) % capacity;
            size++;
            return true;
        }


        T Pop()
        {
            if(IsEmpty()) 
            {
                throw("Queue is Empty!");
            }

            T value = data[head];
            head = (head + 1) % capacity;
            size--;
            return value;
        }

        int Size() const
        {
            return size;
        }
        
};

#endif