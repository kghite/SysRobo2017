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


/*
 * Wrap an angle measurment (in radians) to be in the range 0 to 2pi
 *
 * angle: the angle that you want to wrap
 *
 * return: angle wrapped to the range 0 to 2pi
 */
float wrap_radians(float angle) {

    while (angle >= 2*M_PI) {

        angle -= 2*M_PI;
    }

    while (angle < 0) {

        angle += 2*M_PI;
    }

    return angle;
}


/*
 * Get a relative difference between two radian angles. The relative difference
 * accounts for wrapping around the 2pi cutoff. The two input angles must be
 * between 0 (inclusive) and 2pi (exclusive).
 *
 * ang_1: first angle to get relative diff of
 * ang_2: second angle to get relative diff of
 *
 * return: the relative difference of ang_1 and ang_2
 */
float get_relative_diff_radians(float ang_1, float ang_2) {

    float diff;
    float relative_diff;

    diff = ang_2 - ang_1;
    relative_diff = fmod(diff + M_PI, 2*M_PI) - M_PI; // fmod is modulus

    return relative_diff;
}


/*
 * Clip a float to between a minimum and maximum value.
 *
 * val: the float value to clip
 * min: the minimum value that val can be
 * max: the maximum value that val can be
 *
 * return: val bounded between min and max
 */
float bound_float(float val, float min, float max) {

    if (val < min) {

        val = min;
    }
    else if (val > max) {

        val = max;
    }

    return val;
}
