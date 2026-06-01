#include "trilateration.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "TRILAT";

// Helper: Matrix inversion for 2x2 matrix
static bool matrix_invert_2x2(const float m[2][2], float inv[2][2]) {
    float det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    
    if (fabs(det) < 1.0e-6f) {
        return false;
    }
    
    float det_inv = 1.0f / det;
    inv[0][0] = det_inv * m[1][1];
    inv[0][1] = -det_inv * m[0][1];
    inv[1][0] = -det_inv * m[1][0];
    inv[1][1] = det_inv * m[0][0];
    
    return true;
}

// Helper: Matrix inversion for 3x3 matrix using Cramer's rule
static bool matrix_invert_3x3(const float m[3][3], float inv[3][3]) {
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
              - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
              + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    
    if (fabs(det) < 1.0e-6f) {
        return false;
    }
    
    float det_inv = 1.0f / det;
    
    inv[0][0] = det_inv * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);
    inv[0][1] = det_inv * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);
    inv[0][2] = det_inv * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);
    
    inv[1][0] = det_inv * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);
    inv[1][1] = det_inv * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);
    inv[1][2] = det_inv * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);
    
    inv[2][0] = det_inv * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    inv[2][1] = det_inv * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);
    inv[2][2] = det_inv * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);
    
    return true;
}

// 2D Trilateration using Least Squares
// Based on: https://www.th-luebeck.de/.../TR-2-2015-least-sqaures-with-ToA.pdf
bool trilateration_2d(
    const anchor_t *anchors,
    uint8_t num_anchors,
    position_t *result,
    float *error_estimate)
{
    if (num_anchors < 3 || !anchors || !result) {
        return false;
    }
    
    // Build A matrix and b vector for least squares
    // A = [x1-x0, y1-y0]
    //     [x2-x0, y2-y0]
    //     [x3-x0, y3-y0]
    //     ...
    // b = 0.5 * (d0² - di² + ri² - r0²) where ri² = xi² + yi²
    
    float A[MAX_ANCHORS][2];
    float b[MAX_ANCHORS];
    
    // Reference point (first anchor)
    float x0 = anchors[0].x;
    float y0 = anchors[0].y;
    float d0_sq = anchors[0].distance * anchors[0].distance;
    float r0_sq = x0*x0 + y0*y0;
    
    // Build matrices (skip first anchor)
    for (uint8_t i = 1; i < num_anchors; i++) {
        A[i-1][0] = anchors[i].x - x0;
        A[i-1][1] = anchors[i].y - y0;
        
        float di_sq = anchors[i].distance * anchors[i].distance;
        float ri_sq = anchors[i].x * anchors[i].x + anchors[i].y * anchors[i].y;
        
        b[i-1] = 0.5f * (d0_sq - di_sq + ri_sq - r0_sq);
    }
    
    // Solve using Normal Equations: (A^T * A)^-1 * A^T * b
    // Compute A^T * A
    float ATA[2][2] = {{0}};
    for (uint8_t i = 0; i < num_anchors - 1; i++) {
        for (uint8_t j = 0; j < 2; j++) {
            for (uint8_t k = 0; k < 2; k++) {
                ATA[j][k] += A[i][j] * A[i][k];
            }
        }
    }
    
    // Invert (A^T * A)
    float ATA_inv[2][2];
    if (!matrix_invert_2x2(ATA, ATA_inv)) {
        ESP_LOGE(TAG, "Failed to invert ATA matrix (2D)");
        return false;
    }
    
    // Compute A^T * b
    float ATb[2] = {0};
    for (uint8_t i = 0; i < num_anchors - 1; i++) {
        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }
    
    // Solution: x = (A^T * A)^-1 * A^T * b
    float px = ATA_inv[0][0] * ATb[0] + ATA_inv[0][1] * ATb[1];
    float py = ATA_inv[1][0] * ATb[0] + ATA_inv[1][1] * ATb[1];
    
    result->x = px;
    result->y = py;
    result->z = 0.0f;
    
    // Calculate error estimate (RMSE of calculated vs measured distances)
    if (error_estimate) {
        float error_sum = 0.0f;
        for (uint8_t i = 0; i < num_anchors; i++) {
            float dx = px - anchors[i].x;
            float dy = py - anchors[i].y;
            float dz = anchors[i].z;  // z difference included even in 2D
            float calc_dist = sqrtf(dx*dx + dy*dy + dz*dz);
            float dist_error = calc_dist - anchors[i].distance;
            error_sum += dist_error * dist_error;
        }
        *error_estimate = sqrtf(error_sum / num_anchors);
    }
    
    return true;
}

// 3D Trilateration using Least Squares
bool trilateration_3d(
    const anchor_t *anchors,
    uint8_t num_anchors,
    position_t *result,
    float *error_estimate)
{
    if (num_anchors < 4 || !anchors || !result) {
        return false;
    }
    
    // Similar to 2D but with 3 unknowns (x, y, z)
    // A matrix is (num_anchors-1) x 3
    float A[MAX_ANCHORS][3];
    float b[MAX_ANCHORS];
    
    // Reference point
    float x0 = anchors[0].x;
    float y0 = anchors[0].y;
    float z0 = anchors[0].z;
    float d0_sq = anchors[0].distance * anchors[0].distance;
    float r0_sq = x0*x0 + y0*y0 + z0*z0;
    
    // Build matrices
    for (uint8_t i = 1; i < num_anchors; i++) {
        A[i-1][0] = anchors[i].x - x0;
        A[i-1][1] = anchors[i].y - y0;
        A[i-1][2] = anchors[i].z - z0;
        
        float di_sq = anchors[i].distance * anchors[i].distance;
        float ri_sq = anchors[i].x * anchors[i].x + 
                      anchors[i].y * anchors[i].y + 
                      anchors[i].z * anchors[i].z;
        
        b[i-1] = 0.5f * (d0_sq - di_sq + ri_sq - r0_sq);
    }
    
    // Compute A^T * A
    float ATA[3][3] = {{0}};
    for (uint8_t i = 0; i < num_anchors - 1; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            for (uint8_t k = 0; k < 3; k++) {
                ATA[j][k] += A[i][j] * A[i][k];
            }
        }
    }
    
    // Invert (A^T * A)
    float ATA_inv[3][3];
    if (!matrix_invert_3x3(ATA, ATA_inv)) {
        ESP_LOGE(TAG, "Failed to invert ATA matrix (3D)");
        return false;
    }
    
    // Compute A^T * b
    float ATb[3] = {0};
    for (uint8_t i = 0; i < num_anchors - 1; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            ATb[j] += A[i][j] * b[i];
        }
    }
    
    // Solution
    float px = ATA_inv[0][0] * ATb[0] + ATA_inv[0][1] * ATb[1] + ATA_inv[0][2] * ATb[2];
    float py = ATA_inv[1][0] * ATb[0] + ATA_inv[1][1] * ATb[1] + ATA_inv[1][2] * ATb[2];
    float pz = ATA_inv[2][0] * ATb[0] + ATA_inv[2][1] * ATb[1] + ATA_inv[2][2] * ATb[2];
    
    result->x = px;
    result->y = py;
    result->z = pz;
    
    // Calculate error estimate
    if (error_estimate) {
        float error_sum = 0.0f;
        for (uint8_t i = 0; i < num_anchors; i++) {
            float dx = px - anchors[i].x;
            float dy = py - anchors[i].y;
            float dz = pz - anchors[i].z;
            float calc_dist = sqrtf(dx*dx + dy*dy + dz*dz);
            float dist_error = calc_dist - anchors[i].distance;
            error_sum += dist_error * dist_error;
        }
        *error_estimate = sqrtf(error_sum / num_anchors);
    }
    
    return true;
}

void trilateration_init_anchors(
    anchor_t *anchors,
    uint8_t num_anchors,
    const float anchor_coords[][3])
{
    for (uint8_t i = 0; i < num_anchors; i++) {
        anchors[i].x = anchor_coords[i][0];
        anchors[i].y = anchor_coords[i][1];
        anchors[i].z = anchor_coords[i][2];
        anchors[i].distance = 0.0f;
        anchors[i].last_update_ms = 0;
    }
}

void trilateration_update_distance(
    anchor_t *anchors,
    uint8_t num_anchors,
    uint8_t anchor_id,
    float distance,
    uint32_t timestamp_ms)
{
    if (anchor_id < num_anchors) {
        // Validate distance (reasonable bounds)
        if (distance > 0.0f && distance < 100.0f) {
            anchors[anchor_id].distance = distance;
            anchors[anchor_id].last_update_ms = timestamp_ms;
        } else {
            anchors[anchor_id].last_update_ms = 0;  // Mark as invalid
        }
    }
}

bool trilateration_have_valid_measurements(
    const anchor_t *anchors,
    uint8_t num_anchors,
    uint32_t current_time_ms)
{
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < num_anchors; i++) {
        uint32_t age = current_time_ms - anchors[i].last_update_ms;
        if (age < ANCHOR_DISTANCE_TIMEOUT_MS && anchors[i].distance > 0.0f) {
            valid_count++;
        }
    }
    
    // Need at least 3 for 2D, 4 for 3D
    return valid_count >= 3;
}
