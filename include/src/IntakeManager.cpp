#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include <stack>
#include <iostream>

using namespace pros;
using namespace competition;
using namespace std;

#define logName "/usd/Donut_Manager_Log.txt"

enum Position {DOWN, MIDDLE, TOP};

// Struct to hold position and color
struct Donut {
    Position position;
    bool correctDonut;
};

class Stack {
    // Intern Class variables
    Donut* arr;  // Array to hold Donut structs
    int top;
    int capacity;

public:

    Stack(int size) {
        capacity = size;
        arr = new Donut[capacity];  // Allocate memory for array of Donut structs
        top = -1; // Initialize top index to -1, indicating an empty stack
    }

    ~Stack() {
        delete[] arr; // Deallocate memory when the stack is destroyed
    }

    void push(Position pos, bool col) {
        if (top == capacity - 1) {
            //cout << "Stack Overflow! Cannot push more elements." << endl;
            return;
        }
        top++;
        arr[top].position = pos;
        arr[top].correctDonut = col;
    }

    // Pop the top element (last added element)
    Donut pop_top() {
        if (top == -1) {
            logToSDCard("Stack Underflow! Cannot pop from an empty stack.", logName);
            return {};  // Return invalid Donut if stack is empty
        }
        Donut value = arr[top];
        top--;
        return value;
    }

    // Pop from a specific position in the stack and shift the rest down
    Donut pop(int pop_pos) {
        if (top == -1 || pop_pos > top || pop_pos < 0) {
            logToSDCard("Invalid pop position or stack underflow!", logName);
            return {};  // Return invalid Donut if stack is empty or pop_pos is invalid
        }

        Donut value = arr[pop_pos];

        // Shift all elements above pop_pos down by one
        for (int i = pop_pos; i < top; i++) {
            arr[i] = arr[i + 1];
        }

        top--; // Decrease the stack size
        return value;
    }

    bool isEmpty() {
        return (top == -1);
    }

    // Helper function to display stack contents
    void display() {
        if (isEmpty()) {
            cout << "Stack is empty!" << endl;
            return;
        }
        for (int i = top; i >= 0; i--) {
            cout << "Position: " << arr[i].position << ", Color: " << arr[i].correctDonut << endl;
        }
    }
};

int addDonut(Position position, int color, Stack &stack) {
    stack.push(position, color);
    return 0;
}

/*void donutManagement() {
    Stack donutStack(10);  // Create a stack with a capacity of 10

    // Add some donuts
    addDonut(MIDDLE, 1, donutStack);
    addDonut(TOP, 2, donutStack);
    addDonut(DOWN, 3, donutStack);

    // Display the stack
    donutStack.display();

    // Pop a specific element (from the middle, for example)
    cout << "\nPopping the second element (position 1) from the stack...\n";
    Donut popped = donutStack.pop(1);
    cout << "Popped Donut -> Position: " << popped.position << ", Color: " << popped.correctDonut << endl;

    // Display the stack again after popping
    donutStack.display();
}*/

