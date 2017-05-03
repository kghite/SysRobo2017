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

    float total = 0.0;
    float average = 0.0;
    int num_elements = vec.size();

    for (int i=0; i<num_elements; i++) {
        float curr_val = vec.at(i);

        // Check that curr_val is not Nan or Inf
        if (isfinite(curr_val)) {
            total += curr_val;
        }
}
    average = total / num_elements;
 
    return average;
}
