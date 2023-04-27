#include <inttypes.h>

#define LOOP_ENTRIES 256 // Maximum number of entries in loop predictor table is set to 256
#define WAY 4       // Its a 4 way associative table
#define LOG_IND 6    // Number of bits used to represent an index in the table
#define LOG_WAY 2    // Number of bits used to represent a  way for a given index
#define TAG_SIZE 14  // Number of bits used to represent a tag in the table
#define ITERATION_SIZE 14 // Max Loop Size( max num of iterations) we can predict using this predictor properly
#define AGE 31      // Initial age of the the Loop_Entry is set to 31

class Loop_Entry{
public:
    uint8_t confidence;    // 2-bit counter representing confidence in this prediction for a Loop_Entry
    uint8_t age;           // 8-bit counter representing age of a Loop_Entry
    uint16_t current_iteration;   // Stores the number of iterations seen currently i.e, current size of the loop
    uint16_t past_iteration;      // Stores the number of iterations seen in past i.e, size of the latest predicted loop
    uint16_t tag;            // Tag for a Loop_Entry
    void Loop_Entry_init(){  // Initialisation of a Loop_Entry
        confidence = 0 ;
        current_iteration = 0 ;
        past_iteration = 0 ;
        age = 0 ;
        tag = 0 ;
    }
};

class Loop_Predictor{
private:
    Loop_Entry table[LOOP_ENTRIES]; // Loop Predictor Table Entries
    int index;              // Index used to identify an address in the Loop
    int hit;              // If we get a miss it is set to -1 , else it is set to the value equal to way where we get hit 
    int ptag;             // The tag calculated
    uint8_t seed;

public:
    uint8_t loop_pred; // The prediction value given by the loop predictor for the current loop instruction
    bool is_valid;     // Validity of prediction predicted by this loop predictor

    void init(){   // Initialising the entries of loop predictor
        for (int i = 0; i < LOOP_ENTRIES; i++){
            table[i].Loop_Entry_init();
        }
        seed = 0;
    }
    uint8_t get_pred(uint64_t pc); //   Getting the prediction value for a given program counter
    void update_Loop_Entry(uint8_t taken, uint8_t tage_pred); // Updating a entry based on it's state
};

uint8_t Loop_Predictor::get_pred(uint64_t pc){   // Prediction value for a Given PC
    uint8_t pred = 0 ; //  predicted value initialized to false
    ptag = ((1 << TAG_SIZE) - 1) & (pc >> LOG_IND);       // Tag calculated from the given PC
    index = (pc & ((1 << LOG_IND) - 1)) << LOG_WAY;         // Index calculated from the given PC
    int i = index ;
    while(i < index + WAY){
        // When a matching block is found,we set hit and valid values based on the past and current iterations
        if ((table[i].tag == ptag)&&(table[i].current_iteration + 1 != table[i].past_iteration)){
            loop_pred = 1;
            pred = 1;   // predicted value will be true 
            break ;
        }
        else if((table[i].tag == ptag)&&(table[i].current_iteration + 1 == table[i].past_iteration)){
            loop_pred = 0;
            pred = 0 ; // predicted value will be false
            break ;
        }
        else{
            i++ ;
        }
    }
    if(i < index + WAY){   // In case if match found then updating the required variables 
        is_valid = (table[i].confidence == 3);
        hit = i ;
        return pred ;
    }
    else if(i == index + WAY){ // In case if no matching entry in the table , then updating the required values 
        pred = 0 ;
        hit = -1 ;
        loop_pred = 0 ;
        is_valid = false ;
        return pred ;   // returns false for this case
    }
}

// Updates the predictor table based on the prediction and the actual result of the loop
void Loop_Predictor::update_Loop_Entry(uint8_t taken, uint8_t tage_pred){
    if (hit >= 0){
        Loop_Entry &Loop_Entry = table[index + hit];  // Loop entry is retrived from the predictor table
        if (is_valid && taken != loop_pred){
            // If the predicton was wrong, Loop_Entry will be evicted i.e, free the Loop_Entry
            Loop_Entry.current_iteration = Loop_Entry.confidence = Loop_Entry.age = Loop_Entry.past_iteration = 0;
            return;
        }
        if(is_valid && taken != tage_pred && Loop_Entry.age<AGE){
            Loop_Entry.age++;
        }

        // Updating the iteration values
        Loop_Entry.current_iteration = ((1 << ITERATION_SIZE) - 1) & (Loop_Entry.current_iteration+1);

        // If the num of iterations seen currently is greater than the number of iterations what was seen last time, then free the Loop_Entry variables according to the previous iteration
        if (Loop_Entry.current_iteration > Loop_Entry.past_iteration && Loop_Entry.past_iteration != 0){
            Loop_Entry.confidence = Loop_Entry.age = Loop_Entry.past_iteration = 0;
        }
        else if(Loop_Entry.current_iteration > Loop_Entry.past_iteration && Loop_Entry.past_iteration == 0){
            Loop_Entry.confidence = 0;
        }

        if (!taken)// If incase the loop is not taken
        {
            if (Loop_Entry.current_iteration != Loop_Entry.past_iteration){   
                // Set the newly allocated Loop_Entry
                if (Loop_Entry.past_iteration != 0){
                    Loop_Entry.age = Loop_Entry.past_iteration = 0;  
                }
                // In case if the current and past iterations are same free the Loop_Entry
                else{
                    Loop_Entry.past_iteration = Loop_Entry.current_iteration;
                }
                Loop_Entry.confidence = 0;   
            }
            else{
                // In case if prediction is correct ,then increasing the confidence of the Loop_Entry 
                if (Loop_Entry.confidence < 3){
                    Loop_Entry.confidence++;
                }
                // Loop having less than 3 iterations are not changed
                if ((Loop_Entry.past_iteration > 0) && (Loop_Entry.past_iteration < 3)){
                    Loop_Entry.confidence = Loop_Entry.age = Loop_Entry.past_iteration = 0;
                }
            }
            Loop_Entry.current_iteration = 0;
        }
    }
    // If the branch is taken but there is no Loop_Entry,then we must allocate an Loop_Entry in the table
    else if (taken){
        seed++;
        seed = seed & 3;
        for (int i = 0; i < WAY; i++){
            int j = (seed + i) & 3 ;
            j += index ;
            if (table[j].age > 0){
                table[j].age--; // Decreasing the age of the an Loop_Entry in the table  until it's age is zer
                continue;
            }
            else if (table[j].age == 0){ // If the age is zero, then we assign values to the table entry accordingly
                table[j].tag = ptag;
                table[j].confidence = table[j].past_iteration = 0;
                table[j].current_iteration = 1;
                table[j].age = AGE;
                break;
            } 
        }
    }
}