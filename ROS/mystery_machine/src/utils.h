/*
 * File that holds helper functions!
 */


/*
 * Average a vector of floats
 *
 * vec: vector of floats to be averaged
 *
 * return: the average of all values within vec
 */
float avg(std::vector <float> vec) {

    // Initialize averaging variables
    int num_elements = vec.size();
    float total = 0.0;
    float average = 0.0;

    // Sum all values in the vector
    for (int i=0; i<num_elements; i++) {
        float curr_val = vec.at(i);

        // Check that curr_val is not Nan or Inf
        if (isfinite(curr_val)) {
            total += curr_val;
        }
    }

    // Normalize the total to get the average
    average = total / num_elements;
 
    return average;
}
