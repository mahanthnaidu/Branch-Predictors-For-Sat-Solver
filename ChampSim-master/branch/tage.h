#include "ooo_cpu.h"
#include <vector>
#include <random>

using namespace std;

#define TAG uint16_t
#define Path uint64_t
#define Index uint16_t
#define History uint64_t

#define COUNTER_BITS 3  
#define O_GEHL_ALPHA 1.6
#define BASE_COUNTER_BITS 2 
#define MIN_HISTORY_LENGTH 4
#define RESET_INTERVAL 256000
#define BIMODAL_TABLE_SIZE 16384
#define PATH_HISTORY_BUFFER_LENGTH 32
#define GLOBAL_HISTORY_BUFFER_LENGTH 1024

const uint8_t TAG_BITS[12] = {7, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14, 15};
const uint8_t INDEX_BITS[12] = {10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 9, 9};

/********    MODIFIED FUNCTION     *******/ 
int rand_func(int predicted_component) // Function to be used in update policy to select a component in case of misprediction
{   vector<int> arr;  // The property here implemented is that if j < k then P(j) will be two times the P(k)
    int j = predicted_component ;
    for(int i=0;i<=12;i++){
        arr.push_back(j+1) ;
        j++ ;
        if(j==13){
            break;
        }
    }
    int p = 4096;
    int size = arr.size();
    double total = 0.0;
    vector<double> prob;
    for (int i = 0; i < size; i++){
        prob.push_back(p);
        total = total + prob[i];
        p = p / 2;
    }
    for (int j = 0; j < size; j++){
        prob[j] = prob[j] / total;
    }

    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<int> dist(prob.begin(), prob.end());

    return arr[dist(gen)];   // Returns the component value
}


class Table_Entry{
public:
    TAG tag;         // Tag value of the Table entry
    uint8_t useful;   // Usefullness of the variable which will be in the range 0 to 3
    uint8_t counter;  // Counter value on which the prediction is deicided in the range 0 to 7
};

class Tage{   
private:
    int no_of_branches;                                   // Number of branches predicted after the latest reset
    uint8_t bimodal_table[BIMODAL_TABLE_SIZE];     // Bimodular Table containing counter of the bimodal predictor and it's bimodular predictions 
    class Table_Entry predictor_component_table[12][(1 << 12)]; // Matrix containing entries of bank components along with the component index except the bimodal component
    uint8_t global_history[GLOBAL_HISTORY_BUFFER_LENGTH], path_history[PATH_HISTORY_BUFFER_LENGTH], use_alt_on_na;  // Global history and Path history
    // golbal_history[] stores the global branch history , path_history[] stores the last bits of the last N branch PCs and use_alt_on_na is a 4 bit counter to decide between alternate and provider component prediction
    int component_hist_len[12]; // Length for each bank component used to compute hashes of that bank component
    uint8_t tage_prediction, pred, alternate_prediction; // Final tage prediction , provider component prediction, and alternate prediction
    int predictor_component, alternate_component;           // Provider and alternate component of last branch PC
    int strength;                        // Strength of provider prediction counter of last branch PC

public:
    void initialization()  // Initialisation function
    {
        no_of_branches = 0; // Number of branches initialized to 0
        int j = 0;
        double O_GEHL_ratio = 1; 
        while (j < 12)  // For initialising each entry of every bank component
        {
            int k = 0;
            while (k < (1 << INDEX_BITS[j]))
            {
                predictor_component_table[j][k].tag = 0;
                predictor_component_table[j][k].useful = 0;                           // Initialised to be not useful
                predictor_component_table[j][k].counter = (1 << (COUNTER_BITS - 1)); // Counter values initialized as weakly taken
                k++;
            }
            double var = 0.5 + MIN_HISTORY_LENGTH * O_GEHL_ratio; // O-GEHL in implemnetation
            component_hist_len[j] = int(var); // L(j) = int( L(1)*(alpha**(j-1)) + 0.5 )
            O_GEHL_ratio = O_GEHL_ALPHA * O_GEHL_ratio;      // Geometric Progression
            j++;
        }
        tage_prediction = 0;
        use_alt_on_na = 8;
        j = 0;
        while (j < BIMODAL_TABLE_SIZE) // Each entry value of the bimodal table is initialized as weakly taken
        {
            bimodal_table[j] = (1 << (BASE_COUNTER_BITS - 1)); 
            j++;
        }
        return;
    }

    uint8_t get_prediction(uint64_t ip, int comp) // It returns the prediction value for a given address at a given component
    {
        if (comp != 0) // Checking whether the given component is not bimodal component 
        {
            Index index = get_predictor_index(ip, comp); // Get the component-specific index
            uint8_t weakly_not_taken = 1 << (COUNTER_BITS - 1);
            return predictor_component_table[comp - 1][index].counter >= weakly_not_taken; // If the counter of entry in this component is more than or equal to weakly taken, then the final result will be taken
        }
        else
        {   // Checking if the given component is bimodal component
            uint8_t weakly_not_taken = 1 << (BASE_COUNTER_BITS - 1);
            Index index = ip & (BIMODAL_TABLE_SIZE - 1); // Get the bimodal index
            return bimodal_table[index] >= weakly_not_taken;// If the counter of the entry in bimodal component is more than or equal to weakly not taken, the final result will be taken
        }
    }

    
    void counter_update(uint8_t &counter, int condition, int low, int high) // Based on a given condition, its counter will get updated accordingly 
    {
        if (!condition) // If the condition is not true
        {
            if (counter >= low + 1) // Counter value is more than the least value
            {
                counter--;
            }
        }
        else // If the condition is true 
        {
            if (counter <= high - 1)  // Counter value is less than the highest value 
            {
                counter++;
            }
        }
        return;
    }

    uint8_t predict(uint64_t ip);            // Returns the final prediction from the TAGE predictor
    void update(uint64_t ip, uint8_t taken); // Function to update entries at each branch instruction of the TAGE predictor
    int get_match_below_n(uint64_t ip, int component);              // Finding the hit component below a specified component
    Index get_predictor_index(uint64_t ip, int component);          // Getting the index of an address for a specific component
    TAG get_tag(uint64_t ip, int component);                        // Getting the tag of an address for a specific component
    Path get_path_history_hash(int component);                      // Hash function to insert path history into the tables
    History get_compressed_global_history(int inSize, int outSize); // Compress global history of last 'inSize' branches into 'outSize' by wrapping the history

    Tage() { ; }; // Constructor 
    ~Tage() { ; }; // Destructor
};

uint8_t Tage::predict(uint64_t ip)
{

    predictor_component = get_match_below_n(ip, 13);       // Predictor having the highest index with a hit for the given address
    alternate_component = get_match_below_n(ip, predictor_component); // Predictor having the highest index less than provider component index with a hit for the given address

    // Predictions for the given address from the provider and alternate component to be used later
    alternate_prediction = get_prediction(ip, alternate_component);
    pred = get_prediction(ip, predictor_component);
    switch (predictor_component)
    {
    case 0:
        // This is the default case where predictor component is the bimodal table
        tage_prediction = pred;
        break;
    default:
        // This is the case when the predictor component is not the bimodal table
        Index index = get_predictor_index(ip, predictor_component);
        strength = abs(2 * predictor_component_table[predictor_component - 1][index].counter + 1 - (1 << COUNTER_BITS)) > 1;
        // Use provider component only if USE_ALT_ON_NA < 8 or the provider counter is strong otherwise use alternate component
        if (use_alt_on_na >= 8 && !strength)
        {
            tage_prediction = alternate_prediction;
        }
        else
        {
            tage_prediction = pred;
        }
        break;
    }
    return tage_prediction;
}

void Tage::update(uint64_t ip, uint8_t taken) // Function to update the table entries 
{
    switch (predictor_component)
    {
    case 0:
        // This is the default case where provider component is the bimodal component and the counter gets updated
        counter_update(bimodal_table[ip & (BIMODAL_TABLE_SIZE - 1)], taken, 0, ((1 << BASE_COUNTER_BITS) - 1));
        break;
    default:
        class Table_Entry *entry = &predictor_component_table[predictor_component - 1][get_predictor_index(ip, predictor_component)];
        uint8_t useful = entry->useful;
        // This is the case where provider component is not strong and the final prediction is not same as alternate prediction
        if (!strength && alternate_prediction != pred)
        {
            counter_update(use_alt_on_na, !(pred == taken), 0, 15);
        }
        if (alternate_component > 0) // The case where alternate component is not the bimodal table
        {
            class Table_Entry *alternate_entry = &predictor_component_table[alternate_component - 1][get_predictor_index(ip, alternate_component)];
            if (alternate_component == 0 && useful == 0) // The case where alternate component is the bimodal table
        {
            counter_update(bimodal_table[ip & (BIMODAL_TABLE_SIZE - 1)], taken, 0, ((1 << BASE_COUNTER_BITS) - 1));
        }
        else if (useful == 0)
            {
                counter_update(alternate_entry->counter, taken, 0, ((1 << COUNTER_BITS) - 1));
            }
        }

        // Update on correct prediction
        if (pred != alternate_prediction) // If the final prediction is not same as alternate prediction
        {
            if (pred == taken && (entry->useful < ((1 << 2) - 1)))
            {
                entry->useful++; // If prediction from provider component was correct , entry useful increased 
            }
            else if (pred != taken && use_alt_on_na < 8 && entry->useful > 0)
            {
                entry->useful--;// If the prediction from alternate component is correct , entry useful decreased 
            }
        }
        counter_update(entry->counter, taken, 0, ((1 << COUNTER_BITS) - 1));
        break;
    }

    if (tage_prediction != taken) // When the overall final tage prediction is incorrect
    {   
        int start_component=rand_func(predictor_component);
        int is_entry_Free = 0;
        for (int i = predictor_component + 1; i <= 12; i++) // Checking whether any component greater than the provider component has zero usefullness for the mispredict entry .
        {
            if (predictor_component_table[i - 1][get_predictor_index(ip, i)].useful == 0){
                is_entry_Free = 1;// bool value set to true if a component is found according to the above condition
            }
        }
        if (!is_entry_Free){// If no such components exists , usefullness counter of all components greater than provider component are decreased.
            if(start_component<=12){
                predictor_component_table[start_component - 1][get_predictor_index(ip, start_component)].useful = predictor_component_table[start_component - 1][get_predictor_index(ip, start_component)].useful - 1 ;
            }
        }
        int is_done = 0;// If multiple components are found according to the given comdition, we choose the most probable component and the probability is assigned according to the given function 
        for (int i = start_component; i <= 12; i++){
            class Table_Entry *new_entry = &predictor_component_table[i - 1][get_predictor_index(ip, i)];
            if (!(new_entry->useful)){
                is_done = 1 ;
                new_entry->counter = (1 << (COUNTER_BITS - 1));
                new_entry->tag = get_tag(ip, i);
                new_entry->tag = get_tag(ip, i);
                break;
            }
        }
        if(!is_done && is_entry_Free){
            for(int k = start_component-1 ; k>predictor_component ; k--){
                class Table_Entry *new_entry = &predictor_component_table[k - 1][get_predictor_index(ip, k)];
                if (new_entry->useful == 0){
                    new_entry->counter = (1 << (COUNTER_BITS - 1));
                    new_entry->tag = get_tag(ip, k);
                    break;
                }  
            }
        }
        
    }
    
    for (int i = GLOBAL_HISTORY_BUFFER_LENGTH - 1; i > 0; i--){ // Updation of the global history and the path history accordingly 
        if(i <= PATH_HISTORY_BUFFER_LENGTH - 1){
            path_history[i] = path_history[i - 1];
        }
        global_history[i] = global_history[i - 1];
    }
    path_history[0] = ip & 1;
    global_history[0] = taken;
    no_of_branches++;
    if (no_of_branches % RESET_INTERVAL == 0){ // If the number of instructions reached the limit set, then the number of branches is reset and usefulness is decreased.
        no_of_branches = 0;
        for (int i = 11 ; i>=0; i--){
            int j  = 0 ;
            while(j < (1 << INDEX_BITS[i])){
                predictor_component_table[i][j].useful >>= 1;
                j++ ;
            }
        }
    }
    return;
}

Path Tage::get_path_history_hash(int component)
{
    Path A,A1,A2;A=0;
    int size ;// Size of hash output
    if(component_hist_len[component - 1] >= 16){
        size = 16 ;
    }
    else{
        size = component_hist_len[component - 1] ;
    }
    int i = PATH_HISTORY_BUFFER_LENGTH - 1;
    while(i >= 0){
        A = A << 1 ;
        A = path_history[i] | A ;
        i--;
    }
    uint8_t indx_comp_1=INDEX_BITS[component - 1];
    A1 = (A & ((1 << size) - 1)) & ((1 << indx_comp_1) - 1); // Get the last M bits of A
    A2 = (A & ((1 << size) - 1)) >> indx_comp_1;             // Get the second last M bits of A

    // Using the hash function from the given resource (CBP-4 L-Tage submission)
    A2 = ((A2 << component) & ((1 << indx_comp_1) - 1)) + (A2 >> abs(indx_comp_1 - component));
    A = (((A1 ^ A2) << component) & ((1 << indx_comp_1) - 1)) + ((A1 ^ A2) >> abs(indx_comp_1 - component));
    return (A);
}

History Tage::get_compressed_global_history(int inSize, int outSize)
{   
    int compressed_hist_length = outSize;
    History compressed_hist = 0; // Stores final compressed history
    History temp_history = 0;  // Temorarily stores some bits of history
    int i = 0;
    while(i < inSize)
    {
        if (i % compressed_hist_length == 0)
        {
            compressed_hist ^= temp_history; // XOR current segment into the compressed history
            temp_history = 0;
        }
        temp_history = temp_history<<1;
        temp_history = temp_history | global_history[i]; // Build history bit vector
        i++;
    }
    return compressed_hist ^ temp_history;
}

Index Tage::get_predictor_index(uint64_t ip, int component) //Finding the index of given address in a specific bank component
{
    Path path_history_hash = get_path_history_hash(component); // Hash of path history
    uint8_t index_comp_2 = INDEX_BITS[component - 1];
    // Hash of global history
    History global_history_hash = get_compressed_global_history(component_hist_len[component - 1], index_comp_2);

    return (global_history_hash ^ ip ^ (ip >> (abs(index_comp_2 - component) + 1)) ^ path_history_hash) & ((1 << index_comp_2) - 1);
}

TAG Tage::get_tag(uint64_t ip, int component) // Finding the tag of given address in a specific component
{   int comp_hist_len = component_hist_len[component - 1];
    uint8_t tag_comp_1 = TAG_BITS[component - 1];
    History global_history_hash = get_compressed_global_history(comp_hist_len, tag_comp_1);
    global_history_hash ^= get_compressed_global_history(comp_hist_len, tag_comp_1 - 1);

    return (global_history_hash ^ ip) & ((1 << tag_comp_1) - 1);
}

int Tage::get_match_below_n(uint64_t ip, int component)
{
    int i = component - 1;
    while (i >= 1)
    {

        // get_tag(ip, i) is the hash Function for a bank component i to find the tag
        // get_predictor_index(ip, i) is the hash Function for a bank component i to find the index of the address.

        if (predictor_component_table[i - 1][get_predictor_index(ip, i)].tag == get_tag(ip, i))// Comaprison of tags at different index .
        { 
            return i; // If the tag matches , then the bank i will be the provider component .
        }
        i--;
    }

    return 0; // This is the case where the loop ends without matching any bank component , then the default bimodal predictor will be taken as alternate predictor .
}
