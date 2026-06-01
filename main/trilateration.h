#ifndef TRILATERATION_H
#define TRILATERATION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configuration
#define MAX_ANCHORS 8
#define ANCHOR_DISTANCE_TIMEOUT_MS 5000

// Structures
typedef struct {
    float x, y, z;
} position_t;

typedef struct {
    float x, y, z;
    uint32_t last_update_ms;
    float distance;
} anchor_t;

// 2D Trilateration - Least Squares solution for 4+ anchors
// Returns true if calculation was successful
bool trilateration_2d(
    const anchor_t *anchors,
    uint8_t num_anchors,
    position_t *result,
    float *error_estimate
);

// 3D Trilateration - Least Squares solution for 5+ anchors  
// Returns true if calculation was successful
bool trilateration_3d(
    const anchor_t *anchors,
    uint8_t num_anchors,
    position_t *result,
    float *error_estimate
);

// Initialize anchor positions (should be called once during setup)
void trilateration_init_anchors(
    anchor_t *anchors,
    uint8_t num_anchors,
    const float anchor_coords[][3]
);

// Update distance measurement for an anchor
void trilateration_update_distance(
    anchor_t *anchors,
    uint8_t num_anchors,
    uint8_t anchor_id,
    float distance,
    uint32_t timestamp_ms
);

// Check if enough anchors have valid recent measurements
bool trilateration_have_valid_measurements(
    const anchor_t *anchors,
    uint8_t num_anchors,
    uint32_t current_time_ms
);

#ifdef __cplusplus
}
#endif

#endif // TRILATERATION_H
