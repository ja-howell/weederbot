#include "aruco_detector.h"

ArucoDetector::ArucoDetector(float marker_size, int marker_dictionary_id) {
        this->marker_size = marker_size;
        this->marker_dictionary_id = marker_dictionary_id;
}
