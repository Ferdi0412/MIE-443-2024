/**
 * Use this file to test what stimuli are being triggered.
*/

#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>

enum StimulusState {
    FOLLOWING,
    LOST_TRACK,
    BLOCKED,
    LIFTED,
    FACE_DETECTED
};

