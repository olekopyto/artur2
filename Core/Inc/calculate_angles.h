/*
 * calculate_angles.h
 *
 *  Created on: Jan 20, 2025
 *      Author: matwa
 */

#ifndef INC_CALCULATE_ANGLES_H_
#define INC_CALCULATE_ANGLES_H_

typedef struct {
    float azimuth;
    float elevation;
} AngleResults;

AngleResults calculate_angles(float cos_alpha, float cos_beta) {
    AngleResults results;

    float cos_alpha_squared = cos_alpha * cos_alpha;
    float cos_beta_squared = cos_beta * cos_beta;
    if (cos_alpha_squared + cos_beta_squared > 1.0f) {
        results.azimuth = 0.0f;
        results.elevation = 0.0f;
        return results;
    }


    float x = cos_alpha;
    float y = cos_beta;
    float z = sqrtf(1.0f - cos_alpha_squared - cos_beta_squared);


    results.azimuth = atan2f(x, y) * (180.0f / M_PI);


    results.elevation = atan2f(z, sqrtf(x * x + y * y)) * (180.0f / M_PI);


    return results;
}


#endif /* INC_CALCULATE_ANGLES_H_ */
