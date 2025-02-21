#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "toSDCard.cpp"
#include <stack>
#include <iostream>
#include <string>

using namespace pros;
using namespace competition;
using namespace std;


enum Position {NONE, DOWN, MIDDLE, TOP, TOP2, START};

// Struct to hold position and color
struct Donut {
    Position position;
    bool correctDonut;
    int onScreenPos;
};

class Stack {
    // Intern Class variables
    Donut* arr;  // Array to hold Donut structs
    int top;
    int capacity;
    string logName = "/usd/Donut_Manager_Log_" + getCurrentTimeStamp() + ".txt";

public:

    Stack(int size) {
        capacity = size;
        arr = new Donut[capacity];  // Allocate memory for array of Donut structs
        top = -1; // Initialize top index to -1, indicating an empty stack
    }

    ~Stack() {
        delete[] arr; // Deallocate memory when the stack is destroyed
    }

    void push(Position pos, bool col, int screenPos) {
        if (top == capacity - 1) {
            logToSDCard("Stack Overflow! Cannot push more elements.\n", logName);
            return;
        }
        top++;
        arr[top].position = pos;
        arr[top].correctDonut = col;
        arr[top].onScreenPos = screenPos;
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

    // Gets Donut Value from the Top Donut
    Donut get_Donut_Value() {
        Donut value = arr[top];
        return value;
    }

    // Gets Donut by Array Position
    Donut get_By_ArrPos(int pos) {
        Donut value = arr[pos];
        return value;
    }

    // Gets Donut by Position
    int get_ArrPos_By_Pos(Position pos) {
        int value = -1;
        for(int i; i <= capacity; i++) {
            if(value == -1) {
                value = (arr[i].position = pos) ? arr[i].position : value;
            }
        }
        return value;
    }

    bool isEmpty() {
        return (top == -1);
    }

    // Helper function to display stack contents
    void display() {
        if (isEmpty()) {
            logToSDCard("Stack is empty!\n", logName);
            return;
        }
        for (int i = top; i >= 0; i--) {
            logToSDCard("Position: " + to_string(arr[i].position) + ", Color: " + to_string(arr[i].correctDonut) + "\n", logName);
        }
    }

    // Sets the position of lowest Donut
    void set_Position(Position pos) {
        arr[1].position = pos;
    }

    // Sets the onScreenPos of the center from the Donut
    void update_onScreenPos(int screenPos, Position pos, bool col) {
        Donut value = get_Donut_Value();
        if(value.position == MIDDLE) {
            set_Position(pos);
        } else if (value.position == TOP) {
            pop(1);
        }

        int arrPos = get_ArrPos_By_Pos(pos);
        if(arr[arrPos].onScreenPos > screenPos) {
            arr[arrPos].onScreenPos = screenPos;
        } else if(arr[arrPos].onScreenPos < screenPos - 50) {
            push(pos, col, screenPos);
        }
    }
};

int addDonut(Position position, bool color, int screenPos, Stack &stack) {
    stack.push(position, color, screenPos);
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

